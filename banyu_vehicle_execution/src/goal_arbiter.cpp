#include "banyu_vehicle_execution/goal_arbiter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace banyu_vehicle_execution
{
namespace
{
bool zeroUuid(const GoalUuid & uuid)
{
  return std::all_of(uuid.begin(), uuid.end(), [](uint8_t byte) {return byte == 0;});
}
}
bool GoalIdentity::valid() const noexcept
{
  return !zeroUuid(uuid) && !owner.empty() && generation != 0;
}
GoalArbiter::GoalArbiter(
  const SteadyTimeSource & time_source, uint32_t required_zero_cycles)
: time_source_(time_source),
  required_zero_cycles_(std::max<uint32_t>(1, required_zero_cycles))
{
}
bool GoalArbiter::generationAvailable(const GoalIdentity & identity) const
{
  const auto it = highest_generation_.find(identity.owner);
  return it == highest_generation_.end() || identity.generation > it->second;
}
bool GoalArbiter::validLeaseFor(const GoalIdentity & identity, const Lease & lease) const
{
  const auto now = time_source_.now();
  return identity.valid() && lease.active && lease.goal == identity &&
         lease.receipt_time <= now && lease.deadline > now &&
         lease.deadline > lease.receipt_time;
}
GoalRequest GoalArbiter::requestGoal(
  const GoalIdentity & identity, const std::optional<Lease> & initial_lease)
{
  if (!identity.valid() || !generationAvailable(identity) ||
    (initial_lease && !validLeaseFor(identity, *initial_lease)))
  {
    return {};
  }
  if (state_ == GoalState::ZeroBarrier || state_ == GoalState::Active ||
    state_ == GoalState::Completed || state_ == GoalState::Cancelled ||
    state_ == GoalState::Faulted || state_ == GoalState::LeaseLost ||
    state_ == GoalState::Displaced)
  {
    if (state_ != GoalState::Active || pending_) {
      return {};
    }
    pending_ = GoalHandle{identity};
    pending_lease_ = initial_lease;
    recordTerminal(active_->goal, GoalState::Displaced, StopReason::Handoff);
    displaced_active_ = last_terminal_;
    active_.reset();
    active_lease_.reset();
    zero_cycles_ = 0;
    state_ = GoalState::ZeroBarrier;
    stop_reason_ = StopReason::Handoff;
    zero_latched_ = true;
    highest_generation_[identity.owner] = identity.generation;
    generation_counter_ = std::max(generation_counter_, identity.generation);
    return {GoalAdmission::Pending, *pending_};
  }
  if (state_ != GoalState::Idle) {
    return {};
  }
  active_ = GoalHandle{identity};
  active_lease_ = initial_lease;
  highest_generation_[identity.owner] = identity.generation;
  generation_counter_ = std::max(generation_counter_, identity.generation);
  state_ = GoalState::Active;
  stop_reason_ = StopReason::None;
  zero_latched_ = !active_lease_.has_value();
  return {GoalAdmission::Activated, *active_};
}
bool GoalArbiter::establishLease(const GoalHandle & handle, const Lease & lease)
{
  if (!isCurrent(handle) || !validLeaseFor(handle.goal, lease)) {
    return false;
  }
  active_lease_ = lease;
  zero_latched_ = false;
  return true;
}
bool GoalArbiter::complete(const GoalHandle & handle)
{
  return stopCurrent(handle, StopReason::Completed);
}
bool GoalArbiter::cancel(const GoalHandle & handle)
{
  if (stopPending(handle, StopReason::Cancelled)) {
    return true;
  }
  return stopCurrent(handle, StopReason::Cancelled);
}
bool GoalArbiter::fault(const GoalHandle & handle)
{
  if (stopPending(handle, StopReason::Fault)) {
    return true;
  }
  return stopCurrent(handle, StopReason::Fault);
}
bool GoalArbiter::leaseLost(const GoalHandle & handle)
{
  if (stopPending(handle, StopReason::LeaseLoss)) {
    return true;
  }
  return stopCurrent(handle, StopReason::LeaseLoss);
}
void GoalArbiter::fault()
{
  if (active_) {
    recordTerminal(active_->goal, GoalState::Faulted, StopReason::Fault);
  }
  if (pending_) {
    recordTerminal(pending_->goal, GoalState::Faulted, StopReason::Fault);
  }
  active_.reset();
  active_lease_.reset();
  pending_.reset();
  pending_lease_.reset();
  enterBarrier(StopReason::Fault);
}
bool GoalArbiter::checkLease()
{
  expireGoals();
  return state_ == GoalState::Active && active_lease_.has_value();
}
bool GoalArbiter::recordZeroPublication()
{
  if (expireGoals()) {
    return false;
  }
  if (state_ != GoalState::ZeroBarrier) {
    return false;
  }
  ++zero_cycles_;
  if (zero_cycles_ < required_zero_cycles_) {
    return false;
  }
  if (stop_reason_ == StopReason::Handoff && pending_) {
    if (pending_lease_ && !validLeaseFor(pending_->goal, *pending_lease_)) {
      terminalizePending(StopReason::LeaseLoss);
      return false;
    }
    active_ = pending_;
    active_lease_ = pending_lease_;
    pending_.reset();
    pending_lease_.reset();
    state_ = GoalState::Active;
    stop_reason_ = StopReason::None;
    zero_cycles_ = 0;
    zero_latched_ = !active_lease_.has_value();
    return true;
  }
  state_ = terminalState(stop_reason_);
  zero_cycles_ = 0;
  zero_latched_ = true;
  return true;
}
bool GoalArbiter::reset()
{
  if (state_ != GoalState::Completed && state_ != GoalState::Cancelled &&
    state_ != GoalState::Faulted && state_ != GoalState::LeaseLost &&
    state_ != GoalState::Displaced)
  {
    return false;
  }
  state_ = GoalState::Idle;
  stop_reason_ = StopReason::None;
  active_.reset();
  active_lease_.reset();
  pending_.reset();
  pending_lease_.reset();
  zero_cycles_ = 0;
  zero_latched_ = true;
  return true;
}
bool GoalArbiter::isCurrent(const GoalHandle & handle) const noexcept
{
  return state_ == GoalState::Active && active_ && *active_ == handle;
}
MotionIntent GoalArbiter::selectOutput(
  const GoalHandle & handle, const MotionIntent & requested)
{
  expireGoals();
  if (zero_latched_ || !isCurrent(handle) ||
    !std::isfinite(requested.linear_x) || !std::isfinite(requested.linear_y) ||
    !std::isfinite(requested.linear_z) || !std::isfinite(requested.yaw_rate))
  {
    return zeroIntent();
  }
  return requested;
}
bool GoalArbiter::stopCurrent(const GoalHandle & handle, StopReason reason)
{
  if (!isCurrent(handle)) {
    return false;
  }
  recordTerminal(handle.goal, terminalState(reason), reason);
  active_.reset();
  active_lease_.reset();
  enterBarrier(reason);
  return true;
}
bool GoalArbiter::stopPending(const GoalHandle & handle, StopReason reason)
{
  if (state_ != GoalState::ZeroBarrier || !pending_ || !(*pending_ == handle)) {
    return false;
  }
  terminalizePending(reason);
  return true;
}
void GoalArbiter::enterBarrier(StopReason reason)
{
  state_ = GoalState::ZeroBarrier;
  stop_reason_ = reason;
  zero_cycles_ = 0;
  zero_latched_ = true;
}
void GoalArbiter::recordTerminal(
  const GoalIdentity & identity, GoalState state, StopReason reason)
{
  last_terminal_ = TerminalRecord{identity, state, reason};
}
void GoalArbiter::terminalizePending(StopReason reason)
{
  if (!pending_) {
    return;
  }
  recordTerminal(pending_->goal, terminalState(reason), reason);
  pending_.reset();
  pending_lease_.reset();
  enterBarrier(reason);
}
bool GoalArbiter::expireGoals()
{
  const auto now = time_source_.now();
  if (state_ == GoalState::Active && active_lease_ &&
    (!active_lease_->active || active_lease_->deadline <= now))
  {
    if (active_) {
      recordTerminal(active_->goal, GoalState::LeaseLost, StopReason::LeaseLoss);
    }
    active_.reset();
    active_lease_.reset();
    enterBarrier(StopReason::LeaseLoss);
    return true;
  }
  if (state_ == GoalState::ZeroBarrier && pending_ && pending_lease_ &&
    (!pending_lease_->active || pending_lease_->deadline <= now))
  {
    terminalizePending(StopReason::LeaseLoss);
    return true;
  }
  return false;
}
GoalState GoalArbiter::terminalState(StopReason reason) const noexcept
{
  switch (reason) {
    case StopReason::Completed:
      return GoalState::Completed;
    case StopReason::Cancelled:
      return GoalState::Cancelled;
    case StopReason::Fault:
      return GoalState::Faulted;
    case StopReason::LeaseLoss:
      return GoalState::LeaseLost;
    case StopReason::None:
    case StopReason::Handoff:
      return GoalState::Displaced;
  }
  return GoalState::Faulted;
}
}  // namespace banyu_vehicle_execution
