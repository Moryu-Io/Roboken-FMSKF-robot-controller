
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TeensyDebug.h>

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#include "ArmDrive/AD_task_main.hpp"
#include "FloorDetect/FD_task_main.hpp"
#include "RobotManager/RM_task_main.hpp"
#include "Debug/Debug_task_main.hpp"
#include "Logger/Logger_task_main.hpp"
#include "Utility/util_cache.hpp"
#include "Utility/util_gptimer.hpp"
#include "Utility/util_led.hpp"
#include "VehicleDrive/VD_task_main.hpp"
#include "global_config.hpp"

// RTOS heap
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

// RTOS handle
TaskHandle_t ArmDriveTask_handle     = NULL;
TaskHandle_t VehicleDriveTask_handle = NULL;
TaskHandle_t FloorDetectTask_handle  = NULL;
TaskHandle_t RobotManagerTask_handle = NULL;
TaskHandle_t DebugTask_handle        = NULL;
TaskHandle_t LoggerTask_handle       = NULL;
TaskHandle_t IdleTask_handle         = NULL;

void idle_task(void *params);

void setup() {
  UTIL::SCB_DisableDCache();
  delay(10);
  Serial.begin(460800);
  ///debug.begin(SerialUSB1);
  UTIL::init_LEDpin();
  ADT::prepare_task();
  FDT::prepare_task();
  VDT::prepare_task();
  RMT::prepare_task();
  DEBUG::prepare_task();
  LGT::prepare_task();

  portBASE_TYPE s1;
  s1 = xTaskCreate(FDT::main, "FloorDetect", FDT_STACk_SIZE, NULL, FDT_PRIORITY, &FloorDetectTask_handle);

  // check for creation errors
  if(s1 != pdPASS) {
    Serial.println("Creation problem");
    while(1)
      ;
  }

  s1 = xTaskCreate(VDT::main, "VehicleDrive", VDT_STACk_SIZE, NULL, VDT_PRIORITY, &VehicleDriveTask_handle);
  s1 = xTaskCreate(ADT::main, "ArmDrive", ADT_STACk_SIZE, NULL, ADT_PRIORITY, &ArmDriveTask_handle);
  //s1 = xTaskCreate(RMT::main, "RobotManager", RMT_STACk_SIZE, NULL, RMT_PRIORITY, &RobotManagerTask_handle);
  s1 = xTaskCreate(DEBUG::main, "Debug", DEBUG_STACk_SIZE, NULL, DEBUG_PRIORITY, &DebugTask_handle);
  s1 = xTaskCreate(LGT::main, "Logger", LGT_STACk_SIZE, NULL, LGT_PRIORITY, &LoggerTask_handle);
  s1 = xTaskCreate(idle_task, "Idle", IDLETASK_STACk_SIZE, NULL, IDLETASK_PRIORITY, &IdleTask_handle);

  vTaskStartScheduler();
}

void loop() {
  delay(1000);
}

void idle_task(void *params) {

  while(1) {
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);

  }
}