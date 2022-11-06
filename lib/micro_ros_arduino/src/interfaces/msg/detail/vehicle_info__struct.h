// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_H_
#define INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pos'
#include "interfaces/msg/detail/vehicle_position__struct.h"
// Member 'floor'
#include "interfaces/msg/detail/floor_detection__struct.h"

// Struct defined in msg/VehicleInfo in the package interfaces.
typedef struct interfaces__msg__VehicleInfo
{
  interfaces__msg__VehiclePosition pos;
  interfaces__msg__FloorDetection floor;
  float cam_pitch;
  uint32_t fault;
} interfaces__msg__VehicleInfo;

// Struct for a sequence of interfaces__msg__VehicleInfo.
typedef struct interfaces__msg__VehicleInfo__Sequence
{
  interfaces__msg__VehicleInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__VehicleInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__VEHICLE_INFO__STRUCT_H_
