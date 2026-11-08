#include "arm_kinematics.h"
#include <cmath>
using namespace std;

CartesianReference ArmKinematics::forward(const float q[2]) { //正运动学
  CartesianReference coordinate{};
  coordinate.x = l0 * cos(q[0]) + l1 * cos(q[0] + q[1]);
  coordinate.y = l0 * sin(q[0]) + l1 * sin(q[0] + q[1]);
  return coordinate;
}

bool ArmKinematics::inverse(const CartesianReference& point,
                      const float previous_q[2], int elbow, float q[2]) { //这个point和coordinate一个意思，但是头文件是这个
  if (elbow != 1 && elbow != -1) return false;
  const float r_square = point.x * point.x + point.y * point.y;
  if (!point.itself.isPossibleRadius(r_square)) return false;
  const float d = elbow * point.itself.getSin(r_square);
  const float cos_angle = point.itself.getCos(r_square);
  const float q0 = atan2(point.y, point.x) - atan2(l1 * d,l0 + l1 * cos_angle);
  q[0] = point.itself.eliminateJump(q0, previous_q[0]);
  q[1] = point.itself.eliminateJump(atan2(d,cos_angle), previous_q[1]);
  return true;
}

bool ArmKinematics::lineReachable(const CartesianReference& start,
                                const CartesianReference& end) {
  if (!start.itself.isPossibleRadius(start.x * start.x + start.y * start.y)) return false;
  if (!end.itself.isPossibleRadius(end.x * end.x + end.y * end.y)) return false;

  const float dis_x = end.x - start.x; const float dis_y = end.y - start.y;
  const float line_square = dis_x * dis_x + dis_y * dis_y;

  float u = 0.0;
  if (line_square > 0.0) {
    u = -(start.x * dis_x + start.y * dis_y) / line_square;
    if (u < 0.0) u = 0.0; if (u > 1.0) u = 1.0;
  }
  const float closest_distance = (start.x + u * dis_x) * (start.x + u * dis_x) + (start.y + u * dis_y) * (start.y + u * dis_y);

  if (!start.itself.isPossibleRadius(closest_distance)) return false;
  return true;
}


bool ArmKinematics::derivatives(const CartesianReference& point, const float q[2],
                               float dq[2], float ddq[2]) {
  const float sin_q0 = std::sin(q[0]), cos_q0 = std::cos(q[0]);
  const float sin_q1 = std::sin(q[1]), cos_q1 = std::cos(q[1]);
  const float sin_q0q1 = std::sin(q[0] + q[1]), cos_q0q1 = std::cos(q[0] + q[1]);

  // 奇异值检测
  if (std::fabs(sin_q1) <= min_sin_q1) {
    return false;
  }
  // 雅可比矩阵各项
  const float mat_00 = -l0 * sin_q0 - l1 * sin_q0q1;
  const float mat_01 = -l1 * sin_q0q1;
  const float mat_10 = l0 * cos_q0 + l1 * cos_q0q1;
  const float mat_11 = l1 * cos_q0q1;

  // 行列式
  const float jacobian_det = l1 * l0 * sin_q1;

  // 一阶导（角速度）的等价提取
  const float num_dq0 = (mat_11 * point.dx) - (mat_01 * point.dy);
  const float num_dq1 = (mat_00 * point.dy) - (mat_10 * point.dx);

  dq[0] = num_dq0 / jacobian_det;
  dq[1] = num_dq1 / jacobian_det;

  // ddq = J * ddq + J_dot * dq
  const float total_speed = dq[0] + dq[1];

  // 提取平方项
  const float v0_sq = dq[0] * dq[0];
  const float v1_sq = total_speed * total_speed;

  // 加速度补偿项
  const float term_ax = point.ddx + l0 * cos_q0 * v0_sq + l1 * cos_q0q1 * v1_sq;
  const float term_ay = point.ddy + l0 * sin_q0 * v0_sq + l1 * sin_q0q1 * v1_sq;

  // 二阶导（角加速度）的计算拆解
  const float calc_ax_11 = mat_11 * term_ax;
  const float calc_ay_01 = mat_01 * term_ay;
  const float calc_ax_10 = mat_10 * term_ax;
  const float calc_ay_00 = mat_00 * term_ay;

  ddq[0] = (calc_ax_11 - calc_ay_01) / jacobian_det;
  ddq[1] = (calc_ay_00 - calc_ax_10) / jacobian_det;

  return true;
}
