// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/ArmExecutionProgress.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ARM_EXECUTION_PROGRESS__STRUCT_H_
#define INTERFACES__MSG__DETAIL__ARM_EXECUTION_PROGRESS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'RUNNING'.
enum
{
  interfaces__msg__ArmExecutionProgress__RUNNING = 0
};

/// Constant 'COMPLETED'.
enum
{
  interfaces__msg__ArmExecutionProgress__COMPLETED = 1
};

/// Constant 'ABORTED'.
enum
{
  interfaces__msg__ArmExecutionProgress__ABORTED = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ArmExecutionProgress in the package interfaces.
/**
  * Executor publication time; elapsed_ms follows the actual scaled trajectory.
 */
typedef struct interfaces__msg__ArmExecutionProgress
{
  std_msgs__msg__Header header;
  uint32_t id;
  uint32_t elapsed_ms;
  uint8_t status;
} interfaces__msg__ArmExecutionProgress;

// Struct for a sequence of interfaces__msg__ArmExecutionProgress.
typedef struct interfaces__msg__ArmExecutionProgress__Sequence
{
  interfaces__msg__ArmExecutionProgress * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__ArmExecutionProgress__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__ARM_EXECUTION_PROGRESS__STRUCT_H_
