#ifndef DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
#define DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H

#include <cmath>

class CartesianReference;

// 同向伸直为零位，q1 相对第一杆
class ArmKinematics {
 public:
  static constexpr float l0 = 0.044f; // to the first joint
  static constexpr float l1 = 0.12f; // to the second joint
  static constexpr float min_sin_q1 = 0.01f;  // 雅可比奇异阈值

  static CartesianReference forward(const float q[2]);
  static bool lineReachable(const CartesianReference& start,
                            const CartesianReference& end);
  // elbow 为 +1/-1；q 展开到 previous_q 附近的圈数
  static bool inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]);
  static bool derivatives(const CartesianReference& point, const float q[2],
                          float dq[2], float ddq[2]);
};

class Angle:public ArmKinematics {
private:
  constexpr static float pi = 3.14159265358979323846f;
  constexpr static float double_pi = 2 * pi;
public:
  float getCos(const float r_square)const{ return (r_square - l0 * l0 - l1 * l1) / (2 * l0 * l1); }
  float getSin(const float r_square)const{ return std::sqrt(1.0f - getCos(r_square) * getCos(r_square)); }
  bool isPossibleRadius(const float r_square) const {
    const float cos_angle = getCos(r_square);
    if (1.0 - cos_angle * cos_angle < min_sin_q1 * min_sin_q1) { return false; }
    return true;
  }
  float eliminateJump(const float angle,const float last_angle) const {
    return angle + double_pi * std::round((last_angle - angle) / double_pi);
  }
};


struct CartesianReference {
  float x{}, y{};      // m
  float dx{}, dy{};    // m/s
  float ddx{}, ddy{};  // m/s²
  Angle itself;
};

#endif  // DOUBLE_JOINTED_ROBOTIC_ARM_KINEMATICS_H
