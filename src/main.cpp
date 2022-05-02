
#include <FreeRTOS_TEENSY4.h>
#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <test_msg/msg/robot_info.h>

#include <QuadEncoder.h>

#include "FloorDetect/FD_task_main.hpp"
#include "VehicleDrive/VD_task_main.hpp"
#include "Utility/util_led.hpp"
#include "Utility/util_cache.hpp"

// RTOS heap
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

QuadEncoder myEnc(2, 2, 3, 0);

void setup_encoder(){
  myEnc.setInitConfig();
  myEnc.init();
}

void setup() {
  UTIL::SCB_DisableDCache();
  delay(10);
  Serial.begin(115200);
  setup_encoder();
  UTIL::init_LEDpin();
  FDT::prepare_task();
  VDT::prepare_task();

  portBASE_TYPE s1;
  s1 = xTaskCreate(FDT::main, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  // check for creation errors
  if (s1 != pdPASS) {
    Serial.println("Creation problem");
    while(1);
  }
  
  s1 = xTaskCreate(VDT::main, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  
  vTaskStartScheduler();
}

void loop() {
  delay(1000);
  Serial.printf("Current position value1: %ld\r\n", myEnc.read());

}
