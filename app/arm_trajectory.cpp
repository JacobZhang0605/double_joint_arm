#include "arm_trajectory.h"

bool ArmTrajectory::start(const CartesianReference& start, const CartesianReference& end, float duration_s) { // 开始
  if (running_ ) return false;
  start_ = start;
  end_ = end;
  duration_s_ = duration_s;
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s) {
    CartesianReference result{};

    if (!running_) return result;

    // 双调用插值，二维
    interpolate(start_.x, end_.x, elapsed_s, duration_s_, result.x, result.dx, result.ddx);
    interpolate(start_.y, end_.y, elapsed_s, duration_s_, result.y, result.dy, result.ddy);

    // 结束
    if (elapsed_s >= duration_s_) {
      running_ = false;
      result.x = end_.x;
      result.y = end_.y;
      result.dx = end_.dx;
      result.dy = end_.dy;
      result.ddx = end_.ddx;
      result.ddy = end_.ddy;
    }

    return result;
}

void ArmTrajectory::interpolate(float start, float end, float elapsed_s,
                          float duration_s, float& position, float& velocity, // 这个duration_s和arm.cpp的高度相似了，那个应该是这里的elapsed_s
                          float& acceleration) {
  position = velocity = acceleration = 0.f;
  if (elapsed_s <= 0.0) {
    position = start;
    return;
  }
  if (elapsed_s >= duration_s) {
    position = end;
    return;
  }

  // 归一化
  const float progress = elapsed_s / duration_s;
  const float delta = end - start;

  const float p2 = progress * progress;
  const float p3 = p2 * progress;
  const float reverse = 1.0f - progress;

  const float scalar_smooth = p3 * (10.0f + progress * (6.0f * progress - 15.0f));
  position = start + scalar_smooth * delta;

  const float temporal_factor_velocity = (30.0f * p2 * reverse * reverse) / duration_s;
  velocity = temporal_factor_velocity * delta;

  const float temporal_factor_acceleration = (60.0f * progress * reverse * (1.0f - 2.0f * progress)) / (duration_s * duration_s);
  acceleration = temporal_factor_acceleration * delta;

}

void ArmTrajectory::abort() {
  running_ = false;
}
