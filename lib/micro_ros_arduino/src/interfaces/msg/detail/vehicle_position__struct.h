// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/VehiclePosition.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__VEHICLE_POSITION__STRUCT_H_
#define INTERFACES__MSG__DETAIL__VEHICLE_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/VehiclePosition in the package interfaces.
typedef struct interfaces__msg__VehiclePosition
{
  int32_t x;
  int32_t y;
  float theta;
} interfaces__msg__VehiclePosition;

// Struct for a sequence of interfaces__msg__VehiclePosition.
typedef struct interfaces__msg__VehiclePosition__Sequence
{
  interfaces__msg__VehiclePosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__VehiclePosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__VEHICLE_POSITION__STRUCT_H_
