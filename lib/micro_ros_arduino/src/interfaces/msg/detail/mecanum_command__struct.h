// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/MecanumCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MECANUM_COMMAND__STRUCT_H_
#define INTERFACES__MSG__DETAIL__MECANUM_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MecanumCommand in the package interfaces.
typedef struct interfaces__msg__MecanumCommand
{
  uint32_t cmd;
  uint32_t time;
  uint32_t speed;
} interfaces__msg__MecanumCommand;

// Struct for a sequence of interfaces__msg__MecanumCommand.
typedef struct interfaces__msg__MecanumCommand__Sequence
{
  interfaces__msg__MecanumCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__MecanumCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__MECANUM_COMMAND__STRUCT_H_
