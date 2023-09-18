// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/JointVector.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__JOINT_VECTOR__STRUCT_H_
#define INTERFACES__MSG__DETAIL__JOINT_VECTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'point'
#include "interfaces/msg/detail/joint__struct.h"

/// Struct defined in msg/JointVector in the package interfaces.
typedef struct interfaces__msg__JointVector
{
  interfaces__msg__Joint__Sequence point;
} interfaces__msg__JointVector;

// Struct for a sequence of interfaces__msg__JointVector.
typedef struct interfaces__msg__JointVector__Sequence
{
  interfaces__msg__JointVector * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__JointVector__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__JOINT_VECTOR__STRUCT_H_
