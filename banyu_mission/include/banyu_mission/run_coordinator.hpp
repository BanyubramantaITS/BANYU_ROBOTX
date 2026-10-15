#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <tuple>

namespace banyu_mission
{

class RunCoordinator
{
public:
  enum class Phase
  {
    Idle, Preparing, Ready, Initializing, Active, SurveyComplete, Surfaced,
    Succeeded, Failed, Interrupted
  };

  struct Snapshot
  {
    uint32_t run_id{0};
    uint32_t command_seq{0};
    uint64_t epoch{0};
    Phase phase{Phase::Idle};
    std::string reason;
  };

  enum class Disposition {Accepted, Duplicate, Rejected};

  struct StartResult
  {
    Disposition disposition{Disposition::Rejected};
    std::string reason;
    Snapshot snapshot;
  };

  explicit RunCoordinator(std::string journal_path = {});
  ~RunCoordinator();

  bool beginPreparation();
  bool markReady();
  bool cancelPreparation(std::string reason);
  StartResult requestStart(uint32_t run_id, uint32_t command_seq, bool ready);
  bool completeInitialization(uint64_t epoch);
  bool transition(uint64_t epoch, Phase phase, std::string reason = {});
  StartResult sealReport(
    uint32_t run_id, uint64_t epoch, uint8_t kind, const std::string & report_id,
    const std::string & canonical_payload);
  bool hasSealedReport(uint32_t run_id, uint64_t epoch, uint8_t kind) const;
  Snapshot snapshot() const;

private:
  struct Seal
  {
    std::string report_id;
    std::string canonical_payload;
  };
  using Seals = std::map<std::tuple<uint32_t, uint64_t, uint8_t>, Seal>;
  bool transitionLocked(uint64_t epoch, Phase phase, const std::string & reason);

  mutable std::mutex mutex_;
  Snapshot snapshot_;
  std::map<uint32_t, uint32_t> accepted_runs_;
  std::map<uint32_t, uint64_t> accepted_epochs_;
  Seals seals_;
  uint32_t max_accepted_sequence_{0};
};

}  // namespace banyu_mission
