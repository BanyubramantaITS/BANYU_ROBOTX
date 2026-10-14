#include "banyu_mission/run_coordinator.hpp"

#include <algorithm>
#include <limits>
#include <utility>

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
}  // namespace

RunCoordinator::RunCoordinator(std::string /*journal_path*/)
{
}

RunCoordinator::~RunCoordinator() = default;

bool RunCoordinator::beginPreparation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (livePhase(snapshot_.phase)) {
    return false;
  }
  snapshot_.phase = Phase::Preparing;
  snapshot_.reason.clear();
  max_accepted_sequence_ = 0;
  return true;
}

bool RunCoordinator::markReady()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.phase != Phase::Preparing && snapshot_.phase != Phase::Ready) {
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
  accepted_runs_.emplace(run_id, command_seq);
  accepted_epochs_.emplace(run_id, next.epoch);
  snapshot_ = next;
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
  if (epoch == 0 || epoch != snapshot_.epoch || !permittedTransition(snapshot_.phase, phase)) {
    return false;
  }
  auto next = snapshot_;
  next.phase = phase;
  next.reason = reason;
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
  seals_.emplace(key, Seal{report_id, canonical_payload});
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

}  // namespace banyu_mission
