#ifndef BANYU_VEHICLE_EXECUTION__GOAL_ARBITER_HPP_
#define BANYU_VEHICLE_EXECUTION__GOAL_ARBITER_HPP_
#include "banyu_vehicle_execution/motion_intent.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
namespace banyu_vehicle_execution
{
using SteadyTime = std::chrono::steady_clock::time_point;
class SteadyTimeSource
{
public:
  virtual ~SteadyTimeSource() = default;
  virtual SteadyTime now() const = 0;
};
using GoalUuid = std::array<uint8_t, 16>;
struct GoalIdentity
{
  GoalUuid uuid{};
  std::string owner{};
  uint64_t generation{0};
  bool valid() const noexcept;
  friend bool operator==(const GoalIdentity & lhs, const GoalIdentity & rhs) noexcept
  {
    return lhs.uuid == rhs.uuid && lhs.owner == rhs.owner &&
           lhs.generation == rhs.generation;
  }
};
struct Lease
{
  GoalIdentity goal{};
  SteadyTime receipt_time{};
  SteadyTime deadline{};
  bool active{true};
};
struct GoalHandle
{
  GoalIdentity goal{};
  bool valid() const noexcept {return goal.valid();}
  friend bool operator==(const GoalHandle & lhs, const GoalHandle & rhs) noexcept
  {
    return lhs.goal == rhs.goal;
  }
};
enum class GoalState
{
  Idle,
  Active,
  ZeroBarrier,
  Completed,
  Cancelled,
  Faulted,
  LeaseLost,
  Displaced
};
enum class StopReason
{
  None,
  Handoff,
  Completed,
  Cancelled,
  Fault,
  LeaseLoss
};
enum class GoalAdmission
{
  Activated,
  Pending,
  Rejected
};
struct GoalRequest
{
  GoalAdmission admission{GoalAdmission::Rejected};
  GoalHandle handle{};
  bool accepted() const noexcept {return admission != GoalAdmission::Rejected;}
};
struct TerminalRecord
{
  GoalIdentity goal{};
  GoalState state{GoalState::Idle};
  StopReason reason{StopReason::None};
};
class GoalArbiter
{
public:
  GoalArbiter(const SteadyTimeSource & time_source, uint32_t required_zero_cycles = 2);
  GoalRequest requestGoal(
    const GoalIdentity & identity, const std::optional<Lease> & initial_lease = std::nullopt);
  bool establishLease(const GoalHandle & handle, const Lease & lease);
  bool complete(const GoalHandle & handle);
  bool cancel(const GoalHandle & handle);
  bool fault(const GoalHandle & handle);
  bool leaseLost(const GoalHandle & handle);
  void fault();
  bool recordZeroPublication();
  bool checkLease();
  bool reset();
  bool isCurrent(const GoalHandle & handle) const noexcept;
  bool zeroLatched() const noexcept {return zero_latched_;}
  GoalState state() const noexcept {return state_;}
  StopReason stopReason() const noexcept {return stop_reason_;}
  uint64_t generation() const noexcept {return generation_counter_;}
  uint32_t zeroPublicationCount() const noexcept {return zero_cycles_;}
  uint32_t requiredZeroCycles() const noexcept {return required_zero_cycles_;}
  std::optional<GoalHandle> activeGoal() const noexcept {return active_;}
  std::optional<GoalHandle> pendingGoal() const noexcept {return pending_;}
  std::optional<TerminalRecord> lastTerminal() const noexcept {return last_terminal_;}
  std::optional<TerminalRecord> displacedActive() const noexcept {return displaced_active_;}
  MotionIntent selectOutput(const GoalHandle & handle, const MotionIntent & requested);

private:
  bool validLeaseFor(const GoalIdentity & identity, const Lease & lease) const;
  bool generationAvailable(const GoalIdentity & identity) const;
  bool stopCurrent(const GoalHandle & handle, StopReason reason);
  bool stopPending(const GoalHandle & handle, StopReason reason);
  void enterBarrier(StopReason reason);
  void recordTerminal(
    const GoalIdentity & identity, GoalState state, StopReason reason);
  void terminalizePending(StopReason reason);
  bool expireGoals();
  GoalState terminalState(StopReason reason) const noexcept;
  const SteadyTimeSource & time_source_;
  const uint32_t required_zero_cycles_;
  GoalState state_{GoalState::Idle};
  StopReason stop_reason_{StopReason::None};
  uint64_t generation_counter_{0};
  std::unordered_map<std::string, uint64_t> highest_generation_;
  std::optional<GoalHandle> active_;
  std::optional<Lease> active_lease_;
  std::optional<GoalHandle> pending_;
  std::optional<Lease> pending_lease_;
  std::optional<TerminalRecord> last_terminal_;
  std::optional<TerminalRecord> displaced_active_;
  uint32_t zero_cycles_{0};
  bool zero_latched_{true};
};
}  // namespace banyu_vehicle_execution
#endif  // BANYU_VEHICLE_EXECUTION__GOAL_ARBITER_HPP_
