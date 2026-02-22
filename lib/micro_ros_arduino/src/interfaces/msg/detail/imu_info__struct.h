// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/ImuInfo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__IMU_INFO__STRUCT_H_
#define INTERFACES__MSG__DETAIL__IMU_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ImuInfo in the package interfaces.
typedef struct interfaces__msg__ImuInfo
{
  uint8_t fault;
  float qx;
  float qy;
  float qz;
  float qw;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
} interfaces__msg__ImuInfo;

// Struct for a sequence of interfaces__msg__ImuInfo.
typedef struct interfaces__msg__ImuInfo__Sequence
{
  interfaces__msg__ImuInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__ImuInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__IMU_INFO__STRUCT_H_
