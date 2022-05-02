#ifndef GLOBAL_CONFIG_HPP_
#define GLOBAL_CONFIG_HPP_

#include <Arduino.h>

/************************ RTOS設定 ここから ************************/
#define ADT_STACk_SIZE      (1024)
#define VDT_STACk_SIZE      (1024)
#define FDT_STACk_SIZE      (1024)
#define RMT_STACk_SIZE      (512)
#define IDLETASK_STACk_SIZE (512)

#define ADT_PRIORITY      (2)
#define VDT_PRIORITY      (2)
#define FDT_PRIORITY      (2)
#define RMT_PRIORITY      (2)
#define IDLETASK_PRIORITY (0)

#define ENABLE_FREERTOS_TASK_STACK_PRINT (0) // IdleタスクでRTOSスタックサイズ測定を行うかどうか
/************************ RTOS設定 ここまで ************************/

/************************ DEBUG PRINT設定 ここから ************************/
//#define DEBUG_PRINT_ADT(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_ADT(fmt, ...)

//#define DEBUG_PRINT_FDT(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_FDT(fmt, ...)

//#define DEBUG_PRINT_VDT_IMU(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT_IMU(fmt, ...)
/************************ DEBUG PRINT設定 ここまで ************************/

#endif