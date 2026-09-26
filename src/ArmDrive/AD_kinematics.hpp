#ifndef AD_KINEMATICS_HPP_
#define AD_KINEMATICS_HPP_

#include "AD_joint_base.hpp"

namespace ADT {
namespace Kinematics {

constexpr int JOINT_COUNT = JointAxis::J_NUM;

struct Vec3 {
  float x;
  float y;
  float z;
};

/**
 * @brief 直列5軸アームの幾何パラメータ。
 *
 * origin_from_parent_m / axis_in_parent は、各関節を回す直前の親座標系で
 * 定義する。q は {Yaw, Pitch1, Pitch2, Roll, Pitch3} の仮想関節角[rad]で、
 * 差動機構のモータ角はここへ渡さない。
 */
struct JointParam {
  Vec3  origin_from_parent_m;
  Vec3  axis_in_parent;
  float zero_offset_rad;
};

struct ModelConfig {
  bool       enabled;
  JointParam joint[JOINT_COUNT];
  Vec3       tool_offset_m;
};

struct Result {
  Vec3  tool_position_m;
  Vec3  joint_origin_m[JOINT_COUNT];
  Vec3  joint_axis_world[JOINT_COUNT];
  float position_jacobian_z[JOINT_COUNT]; // dz/dq [m/rad]
};

/**
 * @brief 実機用の幾何設定を返す。
 * @note quinque_model_description.xacro の関節配置とPitch1の-90度変換を使用。
 */
const ModelConfig &robot_model_config();

bool is_valid(const ModelConfig &config);
bool calculate(const ModelConfig &config, const float joint_rad[JOINT_COUNT], Result *result);

} // namespace Kinematics
} // namespace ADT

#endif
