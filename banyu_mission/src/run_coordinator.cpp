#include "banyu_mission/run_coordinator.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace banyu_mission
{
namespace
{
using Phase = RunCoordinator::Phase;

bool livePhase(Phase phase)
{
  return phase == Phase::Initializing || phase == Phase::Active ||
         phase == Phase::SurveyComplete || phase == Phase::Surfaced;
}

bool terminalPhase(Phase phase)
{
  return phase == Phase::Succeeded || phase == Phase::Failed || phase == Phase::Interrupted;
}

bool permittedTransition(Phase current, Phase next)
{
  if (!livePhase(current)) {
    return false;
  }
  if (next == Phase::Failed || next == Phase::Interrupted) {
    return true;
  }
  return (current == Phase::Initializing && next == Phase::Active) ||
         (current == Phase::Active && next == Phase::SurveyComplete) ||
         (current == Phase::SurveyComplete && next == Phase::Surfaced) ||
         (current == Phase::Surfaced && next == Phase::Succeeded);
}

std::string systemError(const std::string & operation)
{
  return operation + ": " + std::strerror(errno);
}
}  // namespace

RunCoordinator::RunCoordinator(std::string journal_path)
: journal_path_(std::move(journal_path))
{
  if (lockJournal()) {
    loadJournal();
  }
}

RunCoordinator::~RunCoordinator()
{
  if (journal_lock_fd_ != -1) {
    ::close(journal_lock_fd_);
  }
}

bool RunCoordinator::lockJournal()
{
  if (journal_path_.empty()) {
    return true;
  }
  journal_lock_fd_ = ::open(
    (journal_path_ + ".lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
  if (journal_lock_fd_ == -1) {
    journal_error_ = systemError("open journal owner lock");
    return false;
  }
  int result;
  do {
    result = ::flock(journal_lock_fd_, LOCK_EX | LOCK_NB);
  } while (result == -1 && errno == EINTR);
  if (result == -1) {
    journal_error_ = systemError("acquire exclusive journal ownership");
    ::close(journal_lock_fd_);
    journal_lock_fd_ = -1;
    return false;
  }
  return true;
}

bool RunCoordinator::beginPreparation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!journal_error_.empty() || livePhase(snapshot_.phase)) {
    return false;
  }
  snapshot_.phase = Phase::Preparing;
  snapshot_.reason.clear();
  // RoboCommand numbers commands per session, so an explicit operator preparation opens a new
  // sequence window. Already accepted run_ids still never restart.
  max_accepted_sequence_ = 0;
  return true;
}

bool RunCoordinator::markReady()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!journal_error_.empty() ||
    (snapshot_.phase != Phase::Preparing && snapshot_.phase != Phase::Ready))
  {
    return false;
  }
  snapshot_.phase = Phase::Ready;
  snapshot_.reason.clear();
  return true;
}

bool RunCoordinator::cancelPreparation(std::string reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.phase != Phase::Preparing && snapshot_.phase != Phase::Ready) {return false;}
  snapshot_.phase = snapshot_.run_id == 0 ? Phase::Idle : Phase::Interrupted;
  snapshot_.reason = std::move(reason);
  if (snapshot_.run_id != 0) {
    std::string error;
    if (!persist(snapshot_, accepted_runs_, error)) {journal_error_ = error;}
  }
  return true;
}

RunCoordinator::StartResult RunCoordinator::requestStart(
  uint32_t run_id, uint32_t command_seq, bool ready)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto result = [this](Disposition disposition, const std::string & reason) {
      return StartResult{disposition, reason, snapshot_};
    };

  if (run_id == 0) {
    return result(Disposition::Rejected, "INVALID_RUN_ID: run_id must be nonzero");
  }
  if (command_seq == 0) {
    return result(Disposition::Rejected, "INVALID_COMMAND_SEQUENCE: sequence must be nonzero");
  }
  if (!journal_error_.empty()) {
    return result(Disposition::Rejected, "JOURNAL_UNAVAILABLE: " + journal_error_);
  }
  const auto previous = accepted_runs_.find(run_id);
  if (previous != accepted_runs_.end()) {
    if (command_seq != previous->second && command_seq <= max_accepted_sequence_) {
      return result(Disposition::Rejected, "COMMAND_REPLAY: run and sequence do not match");
    }
    return result(Disposition::Duplicate, "RUN_ALREADY_ACCEPTED");
  }
  if (command_seq <= max_accepted_sequence_) {
    return result(Disposition::Rejected, "COMMAND_REPLAY: sequence was already accepted");
  }
  if (livePhase(snapshot_.phase)) {
    return result(Disposition::Rejected, "RUN_CONFLICT: another accepted run is live");
  }
  if (!ready || snapshot_.phase != Phase::Ready) {
    return result(Disposition::Rejected, "NOT_READY: autonomous readiness is required");
  }
  if (snapshot_.epoch == std::numeric_limits<uint64_t>::max()) {
    return result(Disposition::Rejected, "EPOCH_EXHAUSTED");
  }

  const Snapshot next{run_id, command_seq, snapshot_.epoch + 1, Phase::Initializing, {}};
  auto accepted = accepted_runs_;
  accepted.emplace(run_id, command_seq);
  auto epochs = accepted_epochs_;
  epochs.emplace(run_id, next.epoch);
  std::string error;
  if (!persist(next, accepted, error, nullptr, &epochs)) {
    journal_error_ = error;
    return result(Disposition::Rejected, "JOURNAL_UNAVAILABLE: " + error);
  }
  snapshot_ = next;
  accepted_runs_ = std::move(accepted);
  accepted_epochs_ = std::move(epochs);
  max_accepted_sequence_ = command_seq;
  return result(Disposition::Accepted, "RUN_ACCEPTED_INITIALIZING");
}

bool RunCoordinator::completeInitialization(uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.phase != Phase::Initializing) {
    return false;
  }
  return transitionLocked(epoch, Phase::Active, {});
}

bool RunCoordinator::transition(uint64_t epoch, Phase phase, std::string reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return transitionLocked(epoch, phase, reason);
}

bool RunCoordinator::transitionLocked(uint64_t epoch, Phase phase, const std::string & reason)
{
  if (!journal_error_.empty() || epoch == 0 || epoch != snapshot_.epoch ||
    !permittedTransition(snapshot_.phase, phase))
  {
    return false;
  }
  if (phase == Phase::Succeeded &&
    seals_.count(std::make_tuple(snapshot_.run_id, epoch, uint8_t{1})) == 0)
  {
    return false;
  }
  auto next = snapshot_;
  next.phase = phase;
  next.reason = reason;
  std::string error;
  if (!persist(next, accepted_runs_, error)) {
    journal_error_ = error;
    snapshot_.phase = Phase::Failed;
    snapshot_.reason = "JOURNAL_UNAVAILABLE: " + error;
    return false;
  }
  snapshot_ = std::move(next);
  return true;
}

RunCoordinator::StartResult RunCoordinator::sealReport(
  uint32_t run_id, uint64_t epoch, uint8_t kind, const std::string & report_id,
  const std::string & canonical_payload)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto result = [this](Disposition disposition, const std::string & reason) {
      return StartResult{disposition, reason, snapshot_};
    };
  if (run_id == 0 || epoch == 0 || (kind != 1 && kind != 2) || report_id.empty() ||
    report_id.size() > 128 || canonical_payload.empty() || canonical_payload.size() > 65536)
  {
    return result(Disposition::Rejected, "INVALID_REPORT_IDENTITY_OR_PAYLOAD");
  }
  const auto key = std::make_tuple(run_id, epoch, kind);
  const auto existing = seals_.find(key);
  if (existing != seals_.end()) {
    const bool identical = existing->second.report_id == report_id &&
      existing->second.canonical_payload == canonical_payload;
    return result(identical ? Disposition::Duplicate : Disposition::Rejected,
             identical ? "REPORT_ALREADY_SEALED" : "REPORT_CONFLICT");
  }
  if (!journal_error_.empty()) {
    return result(Disposition::Rejected, "JOURNAL_UNAVAILABLE: " + journal_error_);
  }
  if (run_id != snapshot_.run_id || epoch != snapshot_.epoch) {
    return result(Disposition::Rejected, "WRONG_RUN");
  }
  if (snapshot_.phase != Phase::Surfaced) {
    return result(Disposition::Rejected, "RUN_NOT_REPORT_READY");
  }
  for (const auto & seal : seals_) {
    if (seal.second.report_id == report_id) {
      return result(Disposition::Rejected, "REPORT_ID_REUSED");
    }
  }
  auto candidate = seals_;
  candidate.emplace(key, Seal{report_id, canonical_payload});
  std::string error;
  if (!persist(snapshot_, accepted_runs_, error, &candidate)) {
    journal_error_ = error;
    snapshot_.phase = Phase::Failed;
    snapshot_.reason = "JOURNAL_UNAVAILABLE: " + error;
    return result(Disposition::Rejected, snapshot_.reason);
  }
  seals_ = std::move(candidate);
  return result(Disposition::Accepted, "REPORT_SEALED_LOCAL_ACCEPTANCE");
}

bool RunCoordinator::hasSealedReport(uint32_t run_id, uint64_t epoch, uint8_t kind) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return seals_.count(std::make_tuple(run_id, epoch, kind)) != 0;
}

RunCoordinator::Snapshot RunCoordinator::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool RunCoordinator::persist(
  const Snapshot & snapshot, const std::map<uint32_t, uint32_t> & accepted_runs,
  std::string & error, const Seals * seals,
  const std::map<uint32_t, uint64_t> * accepted_epochs) const
{
  if (journal_path_.empty()) {
    return true;
  }
  const auto & epochs = accepted_epochs ? *accepted_epochs : accepted_epochs_;
  std::ostringstream output;
  output << "BANYU_RUN_COORDINATOR_V2\n" << snapshot.epoch << ' ' << snapshot.run_id << ' ' <<
    snapshot.command_seq << ' ' << static_cast<int>(snapshot.phase) << '\n' <<
    std::quoted(snapshot.reason) << '\n' << accepted_runs.size() << '\n';
  for (const auto & entry : accepted_runs) {
    const auto epoch = epochs.find(entry.first);
    if (epoch == epochs.end()) {
      error = "accepted run has no recorded epoch";
      return false;
    }
    output << entry.first << ' ' << entry.second << ' ' << epoch->second << '\n';
  }
  const auto & reports = seals ? *seals : seals_;
  output << "SEALS_V1 " << reports.size() << '\n';
  for (const auto & report : reports) {
    output << std::get<0>(report.first) << ' ' << std::get<1>(report.first) << ' ' <<
      static_cast<unsigned>(std::get<2>(report.first)) << ' ' <<
      std::quoted(report.second.report_id) << ' ' <<
      std::quoted(report.second.canonical_payload) << '\n';
  }
  const auto contents = output.str();
  const auto parent = std::filesystem::path(journal_path_).parent_path();
  const auto directory = parent.empty() ? std::filesystem::path(".") : parent;
  std::string template_path = journal_path_ + ".tmp.XXXXXX";
  std::vector<char> temporary(template_path.begin(), template_path.end());
  temporary.push_back('\0');
  const int file = ::mkstemp(temporary.data());
  if (file == -1) {
    error = systemError("create journal temporary file");
    return false;
  }
  bool written = true;
  std::size_t offset = 0;
  while (offset < contents.size()) {
    const auto count = ::write(file, contents.data() + offset, contents.size() - offset);
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      error = systemError("write journal");
      written = false;
      break;
    }
    offset += static_cast<std::size_t>(count);
  }
  if (written && ::fsync(file) != 0) {
    error = systemError("sync journal");
    written = false;
  }
  if (::close(file) != 0 && written) {
    error = systemError("close journal");
    written = false;
  }
  if (!written) {
    ::unlink(temporary.data());
    return false;
  }
  if (::rename(temporary.data(), journal_path_.c_str()) != 0) {
    error = systemError("commit journal");
    ::unlink(temporary.data());
    return false;
  }
  const int directory_fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory_fd == -1) {
    error = systemError("open journal directory");
    return false;
  }
  const bool synced = ::fsync(directory_fd) == 0;
  if (!synced) {
    error = systemError("sync journal directory");
  }
  ::close(directory_fd);
  return synced;
}

void RunCoordinator::loadJournal()
{
  if (journal_path_.empty()) {
    return;
  }
  std::error_code status_error;
  const bool exists = std::filesystem::exists(journal_path_, status_error);
  if (status_error) {
    journal_error_ = "inspect journal: " + status_error.message();
    return;
  }
  if (!exists) {
    return;
  }
  const auto size = std::filesystem::file_size(journal_path_, status_error);
  if (status_error || size > 1024 * 1024) {
    journal_error_ = "journal is unreadable or oversized";
    return;
  }
  std::ifstream input(journal_path_);
  Snapshot loaded;
  std::string format;
  int phase = -1;
  uint64_t run_id = 0;
  uint64_t command_seq = 0;
  std::size_t count = 0;
  if (!(input >> format >> loaded.epoch >> run_id >> command_seq >> phase >>
    std::quoted(loaded.reason) >> count) ||
    (format != "BANYU_RUN_COORDINATOR_V1" && format != "BANYU_RUN_COORDINATOR_V2") ||
    run_id == 0 || run_id > std::numeric_limits<uint32_t>::max() ||
    command_seq == 0 || command_seq > std::numeric_limits<uint32_t>::max() ||
    phase < static_cast<int>(Phase::Initializing) || phase > static_cast<int>(Phase::Interrupted) ||
    count == 0 || count > 100000 || loaded.epoch != count)
  {
    journal_error_ = "invalid journal header";
    return;
  }
  loaded.run_id = static_cast<uint32_t>(run_id);
  loaded.command_seq = static_cast<uint32_t>(command_seq);
  loaded.phase = static_cast<Phase>(phase);
  const bool explicit_epochs = format == "BANYU_RUN_COORDINATOR_V2";
  std::map<uint32_t, uint32_t> accepted;
  std::map<uint32_t, uint32_t> sequences;
  std::map<uint32_t, uint64_t> epochs;
  std::map<uint64_t, uint32_t> runs_by_epoch;
  uint32_t maximum = 0;
  for (std::size_t i = 0; i < count; ++i) {
    uint64_t entry_epoch = 0;
    if (!(input >> run_id >> command_seq) || (explicit_epochs && !(input >> entry_epoch)) ||
      run_id == 0 || run_id > std::numeric_limits<uint32_t>::max() || command_seq == 0 ||
      command_seq > std::numeric_limits<uint32_t>::max() ||
      (explicit_epochs && (entry_epoch == 0 || entry_epoch > count)))
    {
      journal_error_ = "invalid accepted-run journal entry";
      return;
    }
    const auto id = static_cast<uint32_t>(run_id);
    const auto sequence = static_cast<uint32_t>(command_seq);
    // V1 journals ordered runs by globally increasing sequence; V2 records each epoch.
    if (!accepted.emplace(id, sequence).second ||
      (!explicit_epochs && !sequences.emplace(sequence, id).second) ||
      (explicit_epochs && (!epochs.emplace(id, entry_epoch).second ||
      !runs_by_epoch.emplace(entry_epoch, id).second)))
    {
      journal_error_ = "duplicate accepted-run journal identity";
      return;
    }
    maximum = std::max(maximum, sequence);
  }
  if (!explicit_epochs) {
    uint64_t sequence_epoch = 0;
    for (const auto & sequence : sequences) {
      epochs.emplace(sequence.second, ++sequence_epoch);
    }
  }
  input >> std::ws;
  const auto current = accepted.find(loaded.run_id);
  const auto current_epoch = epochs.find(loaded.run_id);
  if (current == accepted.end() || current->second != loaded.command_seq ||
    current_epoch == epochs.end() || current_epoch->second != loaded.epoch ||
    (!explicit_epochs && loaded.command_seq != maximum))
  {
    journal_error_ = "inconsistent accepted-run journal";
    return;
  }
  Seals loaded_seals;
  if (!input.eof()) {
    std::string section;
    std::size_t seal_count = 0;
    if (!(input >> section >> seal_count) || section != "SEALS_V1" || seal_count > count * 2) {
      journal_error_ = "invalid seal journal section";
      return;
    }
    std::map<std::string, bool> report_ids;
    for (std::size_t i = 0; i < seal_count; ++i) {
      uint64_t seal_run = 0;
      uint64_t seal_epoch = 0;
      unsigned kind = 0;
      Seal seal;
      if (!(input >> seal_run >> seal_epoch >> kind >> std::quoted(seal.report_id) >>
        std::quoted(seal.canonical_payload)) || seal_run == 0 ||
        seal_run > std::numeric_limits<uint32_t>::max() || (kind != 1 && kind != 2) ||
        seal.report_id.empty() || seal.report_id.size() > 128 || seal.canonical_payload.empty() ||
        seal.canonical_payload.size() > 65536)
      {
        journal_error_ = "invalid sealed report entry";
        return;
      }
      const auto identity = epochs.find(static_cast<uint32_t>(seal_run));
      if (identity == epochs.end() || identity->second != seal_epoch ||
        !loaded_seals.emplace(
          std::make_tuple(identity->first, seal_epoch, static_cast<uint8_t>(kind)), seal).second ||
        !report_ids.emplace(seal.report_id, true).second)
      {
        journal_error_ = "inconsistent sealed report identity";
        return;
      }
    }
    input >> std::ws;
    if (!input.eof()) {
      journal_error_ = "trailing seal journal contents";
      return;
    }
  }
  accepted_runs_ = std::move(accepted);
  accepted_epochs_ = std::move(epochs);
  max_accepted_sequence_ = explicit_epochs ? loaded.command_seq : maximum;
  seals_ = std::move(loaded_seals);
  snapshot_ = std::move(loaded);
  if (!terminalPhase(snapshot_.phase)) {
    snapshot_.phase = Phase::Interrupted;
    snapshot_.reason = "PROCESS_RESTART: accepted run cannot be resumed";
    std::string error;
    if (!persist(snapshot_, accepted_runs_, error)) {
      journal_error_ = error;
      snapshot_.reason = "JOURNAL_UNAVAILABLE: " + error;
    }
  }
}

}  // namespace banyu_mission
