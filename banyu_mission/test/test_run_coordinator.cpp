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
