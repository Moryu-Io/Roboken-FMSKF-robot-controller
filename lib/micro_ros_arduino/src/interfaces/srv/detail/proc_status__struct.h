// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/ProcStatus.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__PROC_STATUS__STRUCT_H_
#define INTERFACES__SRV__DETAIL__PROC_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ProcStatus in the package interfaces.
typedef struct interfaces__srv__ProcStatus_Request
{
  uint32_t id;
} interfaces__srv__ProcStatus_Request;

// Struct for a sequence of interfaces__srv__ProcStatus_Request.
typedef struct interfaces__srv__ProcStatus_Request__Sequence
{
  interfaces__srv__ProcStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__ProcStatus_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ProcStatus in the package interfaces.
typedef struct interfaces__srv__ProcStatus_Response
{
  uint32_t status;
} interfaces__srv__ProcStatus_Response;

// Struct for a sequence of interfaces__srv__ProcStatus_Response.
typedef struct interfaces__srv__ProcStatus_Response__Sequence
{
  interfaces__srv__ProcStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__ProcStatus_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__PROC_STATUS__STRUCT_H_
