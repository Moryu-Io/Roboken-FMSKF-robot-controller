// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/FloorDetection.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__FLOOR_DETECTION__STRUCT_H_
#define INTERFACES__MSG__DETAIL__FLOOR_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/FloorDetection in the package interfaces.
typedef struct interfaces__msg__FloorDetection
{
  uint8_t right;
  uint8_t left;
  uint8_t forward;
  uint8_t back;
  uint8_t rightforward;
  uint8_t leftforward;
  uint8_t rightback;
  uint8_t leftback;
} interfaces__msg__FloorDetection;

// Struct for a sequence of interfaces__msg__FloorDetection.
typedef struct interfaces__msg__FloorDetection__Sequence
{
  interfaces__msg__FloorDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__FloorDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__FLOOR_DETECTION__STRUCT_H_
