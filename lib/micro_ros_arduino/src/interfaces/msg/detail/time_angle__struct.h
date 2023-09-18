// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/TimeAngle.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__TIME_ANGLE__STRUCT_H_
#define INTERFACES__MSG__DETAIL__TIME_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'arm'
#include "interfaces/msg/detail/joint_vector__struct.h"

/// Struct defined in msg/TimeAngle in the package interfaces.
typedef struct interfaces__msg__TimeAngle
{
  uint32_t id;
  interfaces__msg__JointVector arm[5];
} interfaces__msg__TimeAngle;

// Struct for a sequence of interfaces__msg__TimeAngle.
typedef struct interfaces__msg__TimeAngle__Sequence
{
  interfaces__msg__TimeAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__TimeAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__TIME_ANGLE__STRUCT_H_
