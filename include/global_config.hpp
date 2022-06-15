#ifndef GLOBAL_CONFIG_HPP_
#define GLOBAL_CONFIG_HPP_

#include <Arduino.h>

/************************ RTOS設定 ここから ************************/
#define ADT_STACk_SIZE      (1024)
#define VDT_STACk_SIZE      (1024)
#define FDT_STACk_SIZE      (1024)
#define RMT_STACk_SIZE      (1024)
#define DEBUG_STACk_SIZE    (1024)
#define IDLETASK_STACk_SIZE (512)

#define ADT_PRIORITY      (1)
#define VDT_PRIORITY      (2)
#define FDT_PRIORITY      (1)
#define RMT_PRIORITY      (2)
#define DEBUG_PRIORITY    (0)
#define IDLETASK_PRIORITY (0)

// MSG Bufferサイズ(Msg共用体サイズ何個分のバッファを用意するか)
#define ADT_MSG_REQ_BUFFER_SIZE (3)
#define VDT_MSG_REQ_BUFFER_SIZE (3)

#define ENABLE_FREERTOS_TASK_STACK_PRINT (0) // IdleタスクでRTOSスタックサイズ測定を行うかどうか
/************************ RTOS設定 ここまで ************************/

/************************ DEBUG PRINT設定 ここから ************************/
#include "../src/Debug/Debug_task_main.hpp"
template <typename... Args>
void debug_printf(const char *format, Args const &...args){
    uint16_t u16_print_size = sprintf((char *)DEBUG::EXT_PRINT_BUF, format, args...) + 1;
    if(u16_print_size >= 1024) u16_print_size = 1024;
    DEBUG::print(DEBUG::EXT_PRINT_BUF, u16_print_size);
}

//#define DEBUG_PRINT_ADT(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_ADT(fmt, ...)

//#define DEBUG_PRINT_FDT(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_FDT(fmt, ...)

//#define DEBUG_PRINT_VDT(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT(fmt, ...)

//#define DEBUG_PRINT_VDT_IMU(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT_IMU(fmt, ...)

//#define DEBUG_PRINT_VDT_MOTOR(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT_MOTOR(fmt, ...)
/************************ DEBUG PRINT設定 ここまで ************************/

#endif