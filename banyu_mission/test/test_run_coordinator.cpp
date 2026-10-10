#include <gtest/gtest.h>

#include <filesystem>
#include <atomic>
#include <fstream>
#include <limits>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "banyu_mission/run_coordinator.hpp"

namespace
{
using Coordinator = banyu_mission::RunCoordinator;
using Phase = Coordinator::Phase;
using Disposition = Coordinator::Disposition;

void prepare(Coordinator & coordinator)
{
  ASSERT_TRUE(coordinator.beginPreparation());
  ASSERT_TRUE(coordinator.markReady());
}

void expectSame(
  const Coordinator::Snapshot & actual, const Coordinator::Snapshot & expected)
{
  EXPECT_EQ(actual.run_id, expected.run_id);
  EXPECT_EQ(actual.command_seq, expected.command_seq);
  EXPECT_EQ(actual.epoch, expected.epoch);
  EXPECT_EQ(actual.phase, expected.phase);
  EXPECT_EQ(actual.reason, expected.reason);
}

class JournalFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string pattern = (std::filesystem::temp_directory_path() /
      "banyu-run-coordinator-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const auto result = ::mkdtemp(buffer.data());
    ASSERT_NE(result, nullptr);
    directory_ = result;
    journal_ = (directory_ / "runs.journal").string();
  }

  void TearDown() override
  {
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
    std::filesystem::remove_all(directory_.string() + ".moved", error);
  }

  std::filesystem::path directory_;
  std::string journal_;
};
}  // namespace

TEST(RunCoordinator, RequiresActualReadyPhaseAndNonzeroIdentity)
{
  Coordinator coordinator;
  const auto initial = coordinator.snapshot();
  EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Rejected);
  expectSame(coordinator.snapshot(), initial);
  prepare(coordinator);
  const auto ready = coordinator.snapshot();
  EXPECT_EQ(coordinator.requestStart(0, 1, true).disposition, Disposition::Rejected);
  EXPECT_EQ(coordinator.requestStart(13, 0, true).disposition, Disposition::Rejected);
  EXPECT_EQ(coordinator.requestStart(13, 1, false).disposition, Disposition::Rejected);
  expectSame(coordinator.snapshot(), ready);
  const auto accepted = coordinator.requestStart(13, 1, true);
  ASSERT_EQ(accepted.disposition, Disposition::Accepted);
  EXPECT_EQ(accepted.snapshot.run_id, 13u);
  EXPECT_EQ(accepted.snapshot.command_seq, 1u);
  EXPECT_EQ(accepted.snapshot.epoch, 1u);
  EXPECT_EQ(accepted.snapshot.phase, Phase::Initializing);
}

TEST(RunCoordinator, ReadinessCanBeRevokedBeforeAcceptance)
{
  Coordinator coordinator;
  EXPECT_FALSE(coordinator.markReady());
  prepare(coordinator);
  ASSERT_TRUE(coordinator.beginPreparation());
  EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Rejected);
  ASSERT_TRUE(coordinator.markReady());
  ASSERT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Accepted);
  EXPECT_FALSE(coordinator.beginPreparation());
  EXPECT_FALSE(coordinator.markReady());
}

TEST(RunCoordinator, DuplicateStartPreservesInitializingActiveAndTerminalState)
{
  Coordinator coordinator;
  prepare(coordinator);
  const auto accepted = coordinator.requestStart(13, 10, true);
  ASSERT_EQ(accepted.disposition, Disposition::Accepted);
  for (const auto phase : {Phase::Initializing, Phase::Active, Phase::SurveyComplete,
      Phase::Surfaced, Phase::Succeeded})
  {
    if (phase != Phase::Initializing) {
      if (phase == Phase::Succeeded) {
        ASSERT_EQ(coordinator.sealReport(13, 1, 1, "report-13-1", "payload").disposition,
          Disposition::Accepted);
      }
      ASSERT_TRUE(coordinator.transition(accepted.snapshot.epoch, phase));
    }
    const auto before = coordinator.snapshot();
    EXPECT_EQ(coordinator.requestStart(13, 10, false).disposition, Disposition::Duplicate);
    EXPECT_EQ(coordinator.requestStart(13, 11, false).disposition, Disposition::Duplicate);
    expectSame(coordinator.snapshot(), before);
  }
  EXPECT_FALSE(coordinator.completeInitialization(accepted.snapshot.epoch));
  EXPECT_FALSE(coordinator.transition(accepted.snapshot.epoch, Phase::Active));
}

TEST(RunCoordinator, RejectsConflictingAndInvalidStartsWithoutMutation)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Accepted);
  ASSERT_TRUE(coordinator.completeInitialization(1));
  const auto active = coordinator.snapshot();
  EXPECT_EQ(coordinator.requestStart(0, 11, true).disposition, Disposition::Rejected);
  EXPECT_EQ(coordinator.requestStart(14, 10, true).disposition, Disposition::Rejected);
  EXPECT_EQ(coordinator.requestStart(14, 11, true).disposition, Disposition::Rejected);
  EXPECT_EQ(coordinator.requestStart(13, 9, true).disposition, Disposition::Rejected);
  expectSame(coordinator.snapshot(), active);
}

TEST(RunCoordinator, OperatorPreparationOpensNewSequenceWindow)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(900, 10, true).disposition, Disposition::Accepted);
  ASSERT_TRUE(coordinator.transition(1, Phase::Interrupted, "operator abort"));
  prepare(coordinator);
  // A restarted RoboCommand session numbers commands from 1 again.
  const auto accepted = coordinator.requestStart(2, 1, true);
  ASSERT_EQ(accepted.disposition, Disposition::Accepted);
  EXPECT_EQ(accepted.snapshot.epoch, 2u);
  EXPECT_EQ(accepted.snapshot.run_id, 2u);
  const auto initializing = coordinator.snapshot();
  EXPECT_EQ(coordinator.requestStart(900, 10, true).disposition, Disposition::Duplicate);
  expectSame(coordinator.snapshot(), initializing);
}

TEST(RunCoordinator, InitializationAndAllSubsequentTransitionsAreEpochFenced)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Accepted);
  const auto initializing = coordinator.snapshot();
  EXPECT_FALSE(coordinator.completeInitialization(0));
  EXPECT_FALSE(coordinator.completeInitialization(2));
  EXPECT_FALSE(coordinator.transition(2, Phase::Interrupted));
  expectSame(coordinator.snapshot(), initializing);
  ASSERT_TRUE(coordinator.transition(1, Phase::Failed, "tracker reset unavailable"));
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(14, 11, true).disposition, Disposition::Accepted);
  const auto newer = coordinator.snapshot();
  EXPECT_FALSE(coordinator.completeInitialization(1));
  EXPECT_FALSE(coordinator.transition(1, Phase::Surfaced));
  expectSame(coordinator.snapshot(), newer);
  EXPECT_TRUE(coordinator.completeInitialization(2));
}

TEST(RunCoordinator, ReportingMilestonesCannotSkipSurveyOrSurfacing)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Accepted);
  EXPECT_FALSE(coordinator.transition(1, Phase::Succeeded));
  EXPECT_FALSE(coordinator.transition(1, Phase::Surfaced));
  ASSERT_TRUE(coordinator.completeInitialization(1));
  EXPECT_FALSE(coordinator.transition(1, Phase::Succeeded));
  EXPECT_FALSE(coordinator.transition(1, Phase::Surfaced));
  ASSERT_TRUE(coordinator.transition(1, Phase::SurveyComplete));
  EXPECT_FALSE(coordinator.transition(1, Phase::Succeeded));
  ASSERT_TRUE(coordinator.transition(1, Phase::Surfaced));
  EXPECT_FALSE(coordinator.transition(1, Phase::Succeeded));
  ASSERT_EQ(coordinator.sealReport(13, 1, 1, "report-13-1", "payload").disposition,
    Disposition::Accepted);
  ASSERT_TRUE(coordinator.transition(1, Phase::Succeeded));
  EXPECT_FALSE(coordinator.transition(1, Phase::Active));
  EXPECT_FALSE(coordinator.transition(1, Phase::Interrupted));
}

TEST(RunCoordinator, FailuresAndAbortsRetainIdentityAndReason)
{
  for (const auto terminal : {Phase::Failed, Phase::Interrupted}) {
    for (const auto phase : {Phase::Initializing, Phase::Active, Phase::SurveyComplete,
        Phase::Surfaced})
    {
      Coordinator coordinator;
      prepare(coordinator);
      ASSERT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Accepted);
      if (phase != Phase::Initializing) {
        ASSERT_TRUE(coordinator.completeInitialization(1));
      }
      if (phase == Phase::SurveyComplete || phase == Phase::Surfaced) {
        ASSERT_TRUE(coordinator.transition(1, Phase::SurveyComplete));
      }
      if (phase == Phase::Surfaced) {
        ASSERT_TRUE(coordinator.transition(1, Phase::Surfaced));
      }
      ASSERT_TRUE(coordinator.transition(1, terminal, "test fault"));
      const auto stopped = coordinator.snapshot();
      EXPECT_EQ(stopped.phase, terminal);
      EXPECT_EQ(stopped.run_id, 13u);
      EXPECT_EQ(stopped.epoch, 1u);
      EXPECT_EQ(stopped.reason, "test fault");
      EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Duplicate);
      expectSame(coordinator.snapshot(), stopped);
    }
  }
}

TEST(RunCoordinator, ConcurrentRetransmissionsInitializeExactlyOnce)
{
  Coordinator coordinator;
  prepare(coordinator);
  std::vector<Coordinator::StartResult> results(16);
  std::vector<std::thread> threads;
  for (std::size_t i = 0; i < results.size(); ++i) {
    threads.emplace_back([&coordinator, &results, i]() {
        results[i] = coordinator.requestStart(13, 1, true);
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  std::size_t accepted = 0;
  for (const auto & result : results) {
    accepted += result.disposition == Disposition::Accepted ? 1 : 0;
    EXPECT_TRUE(
      result.disposition == Disposition::Accepted || result.disposition == Disposition::Duplicate);
    EXPECT_EQ(result.snapshot.epoch, 1u);
  }
  EXPECT_EQ(accepted, 1u);
}

TEST_F(JournalFixture, PersistsAcceptanceBeforeInitializationAndRejectsRestartReplay)
{
  {
    Coordinator coordinator(journal_);
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(std::filesystem::exists(journal_));
  }
  Coordinator restarted(journal_);
  const auto interrupted = restarted.snapshot();
  EXPECT_EQ(interrupted.phase, Phase::Interrupted);
  EXPECT_EQ(interrupted.run_id, 13u);
  EXPECT_EQ(interrupted.command_seq, 10u);
  EXPECT_EQ(interrupted.epoch, 1u);
  EXPECT_NE(interrupted.reason.find("PROCESS_RESTART"), std::string::npos);
  EXPECT_EQ(restarted.requestStart(13, 10, true).disposition, Disposition::Duplicate);
  expectSame(restarted.snapshot(), interrupted);
  EXPECT_FALSE(restarted.completeInitialization(1));
  prepare(restarted);
  EXPECT_EQ(restarted.requestStart(13, 1, true).disposition, Disposition::Duplicate);
  const auto next = restarted.requestStart(14, 1, true);
  ASSERT_EQ(next.disposition, Disposition::Accepted);
  EXPECT_EQ(next.snapshot.epoch, 2u);
}

TEST_F(JournalFixture, NewRoboCommandSessionSequencesSurviveRestart)
{
  {
    Coordinator coordinator(journal_);
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(77, 30, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.transition(1, Phase::Failed, "execution failed"));
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(78, 1, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.transition(2, Phase::Failed, "execution failed"));
  }
  Coordinator restarted(journal_);
  const auto failed = restarted.snapshot();
  EXPECT_EQ(failed.phase, Phase::Failed);
  EXPECT_EQ(failed.run_id, 78u);
  EXPECT_EQ(failed.command_seq, 1u);
  EXPECT_EQ(failed.epoch, 2u);
  EXPECT_EQ(restarted.requestStart(77, 30, true).disposition, Disposition::Duplicate);
  EXPECT_EQ(restarted.requestStart(77, 1, true).disposition, Disposition::Rejected);
  EXPECT_EQ(restarted.requestStart(78, 1, true).disposition, Disposition::Duplicate);
  expectSame(restarted.snapshot(), failed);
  prepare(restarted);
  const auto next = restarted.requestStart(79, 1, true);
  ASSERT_EQ(next.disposition, Disposition::Accepted);
  EXPECT_EQ(next.snapshot.epoch, 3u);
}

TEST_F(JournalFixture, LoadsLegacyV1JournalWithSequenceOrderedEpochs)
{
  {
    std::ofstream legacy(journal_);
    legacy << "BANYU_RUN_COORDINATOR_V1\n2 14 11 8\n\"execution failed\"\n2\n13 10\n14 11\n"
      "SEALS_V1 0\n";
  }
  {
    Coordinator coordinator(journal_);
    const auto loaded = coordinator.snapshot();
    EXPECT_EQ(loaded.phase, Phase::Failed);
    EXPECT_EQ(loaded.run_id, 14u);
    EXPECT_EQ(loaded.epoch, 2u);
    EXPECT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Duplicate);
    prepare(coordinator);
    const auto next = coordinator.requestStart(15, 1, true);
    ASSERT_EQ(next.disposition, Disposition::Accepted);
    EXPECT_EQ(next.snapshot.epoch, 3u);
  }
  Coordinator rewritten(journal_);
  EXPECT_EQ(rewritten.snapshot().run_id, 15u);
  EXPECT_EQ(rewritten.snapshot().epoch, 3u);
}

TEST_F(JournalFixture, PersistsTerminalFailureAndAcceptedHistoryAcrossMultipleRestarts)
{
  {
    Coordinator coordinator(journal_);
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(50, 10, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.transition(1, Phase::Interrupted, "operator takeover"));
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(13, 20, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.completeInitialization(2));
    ASSERT_TRUE(coordinator.transition(2, Phase::Failed, "execution failed"));
  }
  for (int restart = 0; restart < 2; ++restart) {
    Coordinator coordinator(journal_);
    const auto failed = coordinator.snapshot();
    EXPECT_EQ(failed.phase, Phase::Failed);
    EXPECT_EQ(failed.reason, "execution failed");
    EXPECT_EQ(failed.epoch, 2u);
    EXPECT_EQ(coordinator.requestStart(50, 10, true).disposition, Disposition::Duplicate);
    EXPECT_EQ(coordinator.requestStart(13, 20, true).disposition, Disposition::Duplicate);
    expectSame(coordinator.snapshot(), failed);
    prepare(coordinator);
    const auto ready = coordinator.snapshot();
    EXPECT_EQ(coordinator.requestStart(50, 1, true).disposition, Disposition::Duplicate);
    EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Duplicate);
    expectSame(coordinator.snapshot(), ready);
  }
}

TEST_F(JournalFixture, MissingJournalDirectoryRejectsWithoutAcceptingRun)
{
  Coordinator coordinator((directory_ / "missing" / "runs.journal").string());
  EXPECT_FALSE(coordinator.beginPreparation());
  const auto initial = coordinator.snapshot();
  const auto rejected = coordinator.requestStart(13, 1, true);
  EXPECT_EQ(rejected.disposition, Disposition::Rejected);
  EXPECT_NE(rejected.reason.find("JOURNAL_UNAVAILABLE"), std::string::npos);
  expectSame(coordinator.snapshot(), initial);
  EXPECT_FALSE(coordinator.markReady());
}

TEST_F(JournalFixture, JournalFailureDuringRunFailsClosed)
{
  Coordinator coordinator(journal_);
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Accepted);
  ASSERT_TRUE(coordinator.completeInitialization(1));
  std::filesystem::rename(directory_, directory_.string() + ".moved");
  EXPECT_FALSE(coordinator.transition(1, Phase::SurveyComplete));
  EXPECT_EQ(coordinator.snapshot().phase, Phase::Failed);
  EXPECT_NE(coordinator.snapshot().reason.find("JOURNAL_UNAVAILABLE"), std::string::npos);
  EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Rejected);
  EXPECT_FALSE(coordinator.beginPreparation());
}

TEST_F(JournalFixture, MalformedAndInconsistentJournalsCannotAuthorizeRuns)
{
  const std::vector<std::string> invalid{
    "truncated",
    "BANYU_RUN_COORDINATOR_V1\n1 0 1 4\n\"\"\n1\n0 1\n",
    "BANYU_RUN_COORDINATOR_V1\n1 13 1 4\n\"\"\n1\n13 2\n",
    "BANYU_RUN_COORDINATOR_V1\n2 13 2 4\n\"\"\n2\n13 2\n13 1\n",
    "BANYU_RUN_COORDINATOR_V1\n2 13 2 4\n\"\"\n2\n13 2\n14 2\n",
    "BANYU_RUN_COORDINATOR_V1\n1 13 1 4\n\"\"\n1\n13 1\ntrailing",
    "BANYU_RUN_COORDINATOR_V1\n1 4294967296 1 4\n\"\"\n1\n13 1\n",
    "BANYU_RUN_COORDINATOR_V1\n1 13 4294967296 4\n\"\"\n1\n13 1\n",
    "BANYU_RUN_COORDINATOR_V1\n1 13 1 2\n\"\"\n1\n13 1\n"
  };
  for (const auto & contents : invalid) {
    SCOPED_TRACE(contents);
    {
      std::ofstream output(journal_);
      output << contents;
    }
    Coordinator coordinator(journal_);
    EXPECT_FALSE(coordinator.beginPreparation());
    EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Rejected);
    EXPECT_EQ(coordinator.snapshot().epoch, 0u);
  }
}

TEST_F(JournalFixture, JournalUsesPrivatePermissionsAndLeavesNoTemporaryArtifacts)
{
  {
    Coordinator coordinator(journal_);
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.completeInitialization(1));
    ASSERT_TRUE(coordinator.transition(1, Phase::Interrupted, "quoted \"reason\"\nnew line"));
  }
  struct stat file_status {};
  ASSERT_EQ(::stat(journal_.c_str(), &file_status), 0);
  EXPECT_EQ(file_status.st_mode & 0777, 0600u);
  std::size_t entries = 0;
  for (const auto & entry : std::filesystem::directory_iterator(directory_)) {
    ++entries;
    EXPECT_TRUE(entry.path() == std::filesystem::path(journal_) ||
      entry.path() == std::filesystem::path(journal_ + ".lock"));
    ASSERT_EQ(::stat(entry.path().c_str(), &file_status), 0);
    EXPECT_EQ(file_status.st_mode & 0777, 0600u);
  }
  EXPECT_EQ(entries, 2u);
  Coordinator restarted(journal_);
  EXPECT_EQ(restarted.snapshot().reason, "quoted \"reason\"\nnew line");
}

TEST_F(JournalFixture, OverlappingOwnerCannotPrepareStartOrModifyAcceptedJournal)
{
  {
    Coordinator owner(journal_);
    prepare(owner);
    ASSERT_EQ(owner.requestStart(13, 10, true).disposition, Disposition::Accepted);
    Coordinator competing(journal_);
    EXPECT_FALSE(competing.beginPreparation());
    EXPECT_FALSE(competing.markReady());
    const auto result = competing.requestStart(14, 11, true);
    EXPECT_EQ(result.disposition, Disposition::Rejected);
    EXPECT_NE(result.reason.find("exclusive journal ownership"), std::string::npos);
    EXPECT_EQ(owner.snapshot().run_id, 13u);
    EXPECT_EQ(owner.snapshot().epoch, 1u);
  }
  Coordinator replacement(journal_);
  EXPECT_EQ(replacement.snapshot().phase, Phase::Interrupted);
  EXPECT_EQ(replacement.snapshot().run_id, 13u);
  EXPECT_EQ(replacement.requestStart(13, 10, true).disposition, Disposition::Duplicate);
  prepare(replacement);
  const auto next = replacement.requestStart(14, 11, true);
  ASSERT_EQ(next.disposition, Disposition::Accepted);
  EXPECT_EQ(next.snapshot.epoch, 2u);
}

TEST_F(JournalFixture, SeparateProcessCannotAcquireLiveOwnerJournal)
{
  Coordinator owner(journal_);
  prepare(owner);
  ASSERT_EQ(owner.requestStart(13, 10, true).disposition, Disposition::Accepted);
  const auto child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    Coordinator competing(journal_);
    const bool denied = !competing.beginPreparation() &&
      competing.requestStart(14, 11, true).disposition == Disposition::Rejected;
    ::_exit(denied ? 0 : 1);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_EQ(owner.snapshot().run_id, 13u);
  EXPECT_TRUE(owner.completeInitialization(1));
}

TEST_F(JournalFixture, OwnerLockMustBeARealWritableFile)
{
  std::filesystem::create_symlink(directory_ / "missing", journal_ + ".lock");
  Coordinator coordinator(journal_);
  EXPECT_FALSE(coordinator.beginPreparation());
  const auto result = coordinator.requestStart(13, 1, true);
  EXPECT_EQ(result.disposition, Disposition::Rejected);
  EXPECT_NE(result.reason.find("journal owner lock"), std::string::npos);
}

TEST(RunCoordinator, CancelledPreparationRequiresExplicitNewPreparation)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_TRUE(coordinator.cancelPreparation("READINESS_LOST"));
  EXPECT_EQ(coordinator.snapshot().phase, Phase::Idle);
  EXPECT_EQ(coordinator.snapshot().reason, "READINESS_LOST");
  EXPECT_FALSE(coordinator.markReady());
  EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Rejected);
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Accepted);
  EXPECT_FALSE(coordinator.cancelPreparation("cannot cancel an accepted run"));
}

TEST(RunCoordinator, MaximumSequenceDoesNotBlockLaterPreparedRuns)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_EQ(
    coordinator.requestStart(13, std::numeric_limits<uint32_t>::max(), true).disposition,
    Disposition::Accepted);
  ASSERT_TRUE(coordinator.transition(1, Phase::Interrupted));
  prepare(coordinator);
  const auto ready = coordinator.snapshot();
  EXPECT_EQ(coordinator.requestStart(13, 1, true).disposition, Disposition::Duplicate);
  expectSame(coordinator.snapshot(), ready);
  EXPECT_EQ(coordinator.requestStart(14, 1, true).disposition, Disposition::Accepted);
}

TEST(RunCoordinator, SealRequiresCurrentSurfacedRunAndImmutableIdentity)
{
  Coordinator coordinator;
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Accepted);
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report", "payload").disposition,
    Disposition::Rejected);
  ASSERT_TRUE(coordinator.completeInitialization(1));
  ASSERT_TRUE(coordinator.transition(1, Phase::SurveyComplete));
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report", "payload").disposition,
    Disposition::Rejected);
  ASSERT_TRUE(coordinator.transition(1, Phase::Surfaced));
  EXPECT_EQ(coordinator.sealReport(14, 1, 1, "report", "payload").disposition,
    Disposition::Rejected);
  EXPECT_EQ(coordinator.sealReport(13, 2, 1, "report", "payload").disposition,
    Disposition::Rejected);
  ASSERT_EQ(coordinator.sealReport(13, 1, 1, "report", "payload").disposition,
    Disposition::Accepted);
  EXPECT_TRUE(coordinator.hasSealedReport(13, 1, 1));
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "other-report", "payload").disposition,
    Disposition::Rejected);
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report", "changed payload").disposition,
    Disposition::Rejected);
  ASSERT_TRUE(coordinator.transition(1, Phase::Interrupted));
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report", "payload").disposition,
    Disposition::Duplicate);
  EXPECT_EQ(coordinator.sealReport(13, 1, 2, "late-delivery", "delivery").disposition,
    Disposition::Rejected);
}

TEST(RunCoordinator, ConcurrentAbortAndSealHaveOneConsistentLinearization)
{
  for (int iteration = 0; iteration < 100; ++iteration) {
    Coordinator coordinator;
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.completeInitialization(1));
    ASSERT_TRUE(coordinator.transition(1, Phase::SurveyComplete));
    ASSERT_TRUE(coordinator.transition(1, Phase::Surfaced));
    std::atomic_bool go{false};
    Coordinator::StartResult seal;
    std::thread reporting([&]() {
        while (!go.load()) {std::this_thread::yield();}
        seal = coordinator.sealReport(13, 1, 1, "report", "payload");
      });
    std::thread aborting([&]() {
        while (!go.load()) {std::this_thread::yield();}
        coordinator.transition(1, Phase::Interrupted, "abort");
      });
    go.store(true);
    reporting.join();
    aborting.join();
    EXPECT_EQ(coordinator.snapshot().phase, Phase::Interrupted);
    const bool committed = seal.disposition == Disposition::Accepted;
    EXPECT_EQ(coordinator.hasSealedReport(13, 1, 1), committed);
    EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report", "payload").disposition,
      committed ? Disposition::Duplicate : Disposition::Rejected);
  }
}

TEST_F(JournalFixture, SealSurvivesTerminalRestartAndCannotBeReattributedToNewRun)
{
  {
    Coordinator coordinator(journal_);
    prepare(coordinator);
    ASSERT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Accepted);
    ASSERT_TRUE(coordinator.completeInitialization(1));
    ASSERT_TRUE(coordinator.transition(1, Phase::SurveyComplete));
    ASSERT_TRUE(coordinator.transition(1, Phase::Surfaced));
    ASSERT_EQ(coordinator.sealReport(13, 1, 1, "report-13-1", "original\npayload").disposition,
      Disposition::Accepted);
    ASSERT_TRUE(coordinator.transition(1, Phase::Succeeded));
  }
  Coordinator coordinator(journal_);
  ASSERT_EQ(coordinator.snapshot().phase, Phase::Succeeded);
  ASSERT_TRUE(coordinator.hasSealedReport(13, 1, 1));
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report-13-1", "original\npayload").disposition,
    Disposition::Duplicate);
  prepare(coordinator);
  ASSERT_EQ(coordinator.requestStart(14, 11, true).disposition, Disposition::Accepted);
  ASSERT_TRUE(coordinator.completeInitialization(2));
  ASSERT_TRUE(coordinator.transition(2, Phase::SurveyComplete));
  ASSERT_TRUE(coordinator.transition(2, Phase::Surfaced));
  EXPECT_EQ(coordinator.sealReport(13, 1, 1, "report-13-1", "original\npayload").disposition,
    Disposition::Duplicate);
  EXPECT_EQ(coordinator.sealReport(14, 2, 1, "report-13-1", "original\npayload").disposition,
    Disposition::Rejected);
  EXPECT_FALSE(coordinator.hasSealedReport(14, 2, 1));
}

TEST_F(JournalFixture, ReadsExistingJournalWithoutSealSection)
{
  {
    std::ofstream legacy(journal_);
    legacy << "BANYU_RUN_COORDINATOR_V1\n1 13 10 8\n\"legacy failure\"\n1\n13 10\n";
  }
  Coordinator coordinator(journal_);
  EXPECT_EQ(coordinator.snapshot().phase, Phase::Failed);
  EXPECT_EQ(coordinator.requestStart(13, 10, true).disposition, Disposition::Duplicate);
  EXPECT_FALSE(coordinator.hasSealedReport(13, 1, 1));
  prepare(coordinator);
  EXPECT_EQ(coordinator.requestStart(14, 11, true).disposition, Disposition::Accepted);
}
