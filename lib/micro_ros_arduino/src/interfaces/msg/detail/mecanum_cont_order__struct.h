// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/MecanumContOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MECANUM_CONT_ORDER__STRUCT_H_
#define INTERFACES__MSG__DETAIL__MECANUM_CONT_ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'speed'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/MecanumContOrder in the package interfaces.
typedef struct interfaces__msg__MecanumContOrder
{
  geometry_msgs__msg__Twist speed;
  uint32_t time_ms;
} interfaces__msg__MecanumContOrder;

// Struct for a sequence of interfaces__msg__MecanumContOrder.
typedef struct interfaces__msg__MecanumContOrder__Sequence
{
  interfaces__msg__MecanumContOrder * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__MecanumContOrder__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__MECANUM_CONT_ORDER__STRUCT_H_
