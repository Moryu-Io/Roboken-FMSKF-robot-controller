#include "AD_kinematics.hpp"

#include <math.h>

namespace ADT {
namespace Kinematics {
namespace {

struct Mat3 {
  float m[3][3];
};

constexpr float AXIS_EPSILON = 1.0e-6f;

Vec3 add(const Vec3 &a, const Vec3 &b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 scale(const Vec3 &v, float s) {
  return {v.x * s, v.y * s, v.z * s};
}

float dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 normalized(const Vec3 &v) {
  const float norm = sqrtf(dot(v, v));
  return scale(v, 1.0f / norm);
}

Mat3 identity() {
  return {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
}

Vec3 multiply(const Mat3 &m, const Vec3 &v) {
  return {
      m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
      m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
      m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z,
  };
}

Mat3 multiply(const Mat3 &a, const Mat3 &b) {
  Mat3 result = {};
  for(int row = 0; row < 3; ++row) {
    for(int col = 0; col < 3; ++col) {
      for(int k = 0; k < 3; ++k) {
        result.m[row][col] += a.m[row][k] * b.m[k][col];
      }
    }
  }
  return result;
}

Mat3 axis_angle(const Vec3 &unit_axis, float angle_rad) {
  const float c = cosf(angle_rad);
  const float s = sinf(angle_rad);
  const float d = 1.0f - c;
  const float x = unit_axis.x;
  const float y = unit_axis.y;
  const float z = unit_axis.z;
  return {{{c + x * x * d, x * y * d - z * s, x * z * d + y * s},
           {y * x * d + z * s, c + y * y * d, y * z * d - x * s},
           {z * x * d - y * s, z * y * d + x * s, c + z * z * d}}};
}

bool finite_vec(const Vec3 &v) {
  return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// quinque_model_description.xacro の base_link -> top_arm_shinai_1。
// TimeAngle/Teensy の Pitch1 は URDF の j2_pitch1 より +90deg 進んだ角度。
// ROS 側と同様に q_urdf = q_timeangle - pi/2 として FK に渡す。
const ModelConfig ROBOT_MODEL_CONFIG = {
    true,
    {
        {{0.0f, 0.0f, 0.1875f}, {0.0f, 0.0f, 1.0f}, 0.0f},
        {{0.0f, -0.0225f, 0.254f}, {0.0f, 1.0f, 0.0f}, -1.5707963267948966f},
        {{0.106f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.0f},
        {{0.021f, 0.0225f, 0.0f}, {1.0f, 0.0f, 0.0f}, 0.0f},
        {{0.163f, -0.015f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.0f},
    },
    {0.316f, 0.015f, 0.0f},
};

} // namespace

const ModelConfig &robot_model_config() {
  return ROBOT_MODEL_CONFIG;
}

bool is_valid(const ModelConfig &config) {
  if(!config.enabled || !finite_vec(config.tool_offset_m)) return false;
  for(int i = 0; i < JOINT_COUNT; ++i) {
    const JointParam &joint = config.joint[i];
    if(!finite_vec(joint.origin_from_parent_m) || !finite_vec(joint.axis_in_parent) || !isfinite(joint.zero_offset_rad)) return false;
    if(dot(joint.axis_in_parent, joint.axis_in_parent) <= AXIS_EPSILON * AXIS_EPSILON) return false;
  }
  return true;
}

bool calculate(const ModelConfig &config, const float joint_rad[JOINT_COUNT], Result *result) {
  if(result == nullptr || joint_rad == nullptr || !is_valid(config)) return false;

  Mat3 rotation = identity();
  Vec3 position = {0.0f, 0.0f, 0.0f};
  for(int i = 0; i < JOINT_COUNT; ++i) {
    if(!isfinite(joint_rad[i])) return false;
    const JointParam &joint = config.joint[i];
    const Vec3 axis_parent = normalized(joint.axis_in_parent);
    position = add(position, multiply(rotation, joint.origin_from_parent_m));
    result->joint_origin_m[i] = position;
    result->joint_axis_world[i] = multiply(rotation, axis_parent);
    rotation = multiply(rotation, axis_angle(axis_parent, joint_rad[i] + joint.zero_offset_rad));
  }

  result->tool_position_m = add(position, multiply(rotation, config.tool_offset_m));
  for(int i = 0; i < JOINT_COUNT; ++i) {
    result->position_jacobian_z[i] = cross(result->joint_axis_world[i],
                                             add(result->tool_position_m, scale(result->joint_origin_m[i], -1.0f))).z;
  }
  return true;
}

} // namespace Kinematics
} // namespace ADT
