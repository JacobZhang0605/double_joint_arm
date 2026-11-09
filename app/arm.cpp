//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"

#include <cmath>

#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"

constexpr float rad2deg = 180 / 3.14159265358979323846;
constexpr float deg2rad = 1 / rad2deg;

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}

bool Arm::feedbackReady() {
  if (joint0_.connect_.check() && joint1_.connect_.check() && joint0_.mode_ != Motor::INIT && joint1_.mode_ != Motor::INIT) return true;
  return false;
}

bool Arm::withinLimits(const float q[2]) const { //形参没了？？？加了个q <- 没事了
  if ( q[0] >= limits_.q_min[0] && q[0] <= limits_.q_max[0] && q[1] >= limits_.q_min[1] && q[1] <= limits_.q_max[1]) return true;
  return false;
}

// 仅失能且反馈有效时按已知姿态标定
bool Arm::calibrate(float q0_rad, float q1_rad) {
  float q[2] = {q0_rad, q1_rad};
  if (mode_ != Mode::Disabled || withinLimits(q) == false) return false;
  // 标定时不要让 CAN 回调改写反馈
  taskENTER_CRITICAL();
  if (feedbackReady() == false) {
    taskEXIT_CRITICAL();
    return false;
  }
  joint0_.resetFeedbackAngle(rad2deg * q0_rad);
  joint1_.resetFeedbackAngle(rad2deg * q1_rad);
  calibrated_ = true;
  taskEXIT_CRITICAL();
  trajAbort();
  updateFeedback();
  reference_ = Reference();
  reference_.q[0] = state_.q[0]; reference_.q[1] = state_.q[1];
  return true;
}

bool Arm::enable() {
  if (calibrated_ == false || feedbackReady() == false) return false;
  if (mode_ == Mode::Joint) return true;
  updateFeedback();
  reference_ = Reference();
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  joint0_.spid_.reset();joint0_.ppid_.reset();joint1_.spid_.reset();joint1_.ppid_.reset();
  mode_ = Mode::Joint;
  return true;
}

void Arm::disable() {
  trajAbort();
  mode_ = Mode::Disabled;
  if (joint0_.mode_ != Motor::INIT) joint0_.mode_ = Motor::POWEROFF;
  if (joint1_.mode_ != Motor::INIT) joint1_.mode_ = Motor::POWEROFF;
}

bool Arm::setReference(const Reference& reference) {
  return applyReference(reference);
}

bool Arm::applyReference(const Reference& reference) {
  if (withinLimits(reference.q) == true) {
    reference_ = reference;
    return true;
  }
  return false;
}

bool Arm::addJointReference(float speed0, float speed1, float dt_s) {
  if (mode_ != Mode::Joint || trajRunning()) return false;
  float speed[2];
  speed[0] = speed0; speed[1] = speed1;
  Reference next = reference_;
  for (int i = 0; i < 2; ++i) {
    const float next_q = next.q[i] + speed[i] * dt_s;
    next.q[i] = (next_q >= limits_.q_max[i]) ? limits_.q_max[i] :
                (next_q <= limits_.q_min[i]) ? limits_.q_min[i] : next_q;
    next.dq[i] = speed[i];
    if ((speed[i] > 0.0f && next_q >= limits_.q_max[i]) || (speed[i] < 0.0f && next_q <= limits_.q_min[i])) {
      next.dq[i] = 0.0f;
    }
    next.ddq[i] = 0.0f;
  }
  return applyReference(next);
}

bool Arm::trajectoryReady() {
  if (mode_ == Mode::Joint && feedbackReady() && calibrated_  && !trajRunning()) return true; //joint模式，可反馈，可校准，然后不动
  return false;
}

bool Arm::trajStart(float x_m, float y_m, float duration_s) {
  if (!trajectoryReady()) return false;
  CartesianReference start = ArmKinematics::forward(reference_.q);
  CartesianReference end;
  end.x = x_m; end.y = y_m;
  if (!withinLimits(reference_.q) || !ArmKinematics::lineReachable(start, end)) return false;
  if (!trajectory_.start(start, end, duration_s)) return false;
  trajectory_elbow_ = sin(reference_.q[1]) > 0.0 ? 1 : -1;
  trajectory_type_ = TrajectoryType::Cartesian;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_finished_ = false;
  return true;
}

// 从实际反馈开始的关节空间同步点到点，目标为累计角度 rad
bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s) {
  const float end[2] = {q0_rad, q1_rad};
  if (!trajectoryReady() || !withinLimits(end)) return false;
  updateFeedback();
  if (!withinLimits(state_.q)) return false;
  reference_ = Reference();
  for (int i = 0; i < 2; ++i) {
    joint_trajectory_start_[i] = state_.q[i];
    joint_trajectory_end_[i] = end[i];
    reference_.q[i] = state_.q[i];
  }
  joint_trajectory_duration_ = duration_s;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_type_ = TrajectoryType::Joint;
  trajectory_finished_ = false;
  return true;
}

void Arm::trajAbort() {
  trajectory_.abort();
  for (int i = 0; i < 2; ++i) {
    reference_.dq[i] = 0.0; reference_.ddq[i] = 0.0;
  }
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  trajectory_start_tick_ = HAL_GetTick();
}

void Arm::trajectoryPlanner() {
  if (!trajRunning()) return;
  float now_tick = HAL_GetTick();
  float duration_s = (now_tick - trajectory_start_tick_) / 1000;
  if (trajectory_type_ == TrajectoryType::Joint) {
    Reference new_reference;
    for (int i = 0; i < 2; ++i) {
      ArmTrajectory::interpolate(joint_trajectory_start_[i],
                                 joint_trajectory_end_[i], duration_s,
                                 joint_trajectory_duration_, new_reference.q[i],
                                 new_reference.dq[i], new_reference.ddq[i]);
    }
    if (!applyReference(new_reference)) {
      trajectory_finished_ = false;
      trajAbort();
    }
    else if (duration_s >= joint_trajectory_duration_) { // 结束
      trajectory_finished_ = true;
      trajectory_type_ = TrajectoryType::None;
    }
    return;
  }
  const CartesianReference point = trajectory_.sample(duration_s);
  Reference new_reference;
  if (!ArmKinematics::inverse(point, reference_.q, trajectory_elbow_, new_reference.q) || !ArmKinematics::derivatives(point, new_reference.q, new_reference.dq, new_reference.ddq)
    || !applyReference(new_reference)) {
    trajectory_finished_ = false;
    trajAbort();
  }
  else if (!trajectory_.running()) {
    trajectory_type_ = TrajectoryType::None;
    trajectory_finished_ = true;
  }
}

void Arm::updateFeedback() {
  if (!feedbackReady()) return;
  const float th0 = joint0_.realAngle();
  const float th1 = joint1_.realAngle();
  const float dth0 = joint0_.realSpeed();
  const float dth1 = joint1_.realSpeed();
  state_.q[0] = th0 * deg2rad;
  state_.q[1] = th1 * deg2rad;
  state_.dq[0] = dth0 * deg2rad;
  state_.dq[1] = dth1 * deg2rad;
}

void Arm::handle() {
  updateFeedback();
  if (!feedbackReady()) {
    calibrated_ = false;
    disable();
    return;
  }
  if (mode_ == Mode::Disabled) return;
  trajectoryPlanner();
  writeMotorCommands();
}

void Arm::writeMotorCommands() {
  joint0_.method_ = Motor::ControlMethod::POSITION_SPEED; joint0_.mode_ = Motor::Mode::WORKING;
  joint1_.method_ = Motor::ControlMethod::POSITION_SPEED; joint1_.mode_ = Motor::Mode::WORKING;
  // 需要传入角，角速度，角加速度
  const float th0 = reference_.q[0] * rad2deg;
  const float th1 = reference_.q[1] * rad2deg;
  const float dth0 = reference_.dq[0] * rad2deg;
  const float dth1 = reference_.dq[1] * rad2deg;
  joint0_.setAngleSpeed(th0, dth0, feedforward_.k_acc[0] * reference_.ddq[0]);
  joint1_.setAngleSpeed(th1, dth1, feedforward_.k_acc[1] * reference_.ddq[1]);
}
