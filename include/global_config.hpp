#ifndef GLOBAL_CONFIG_HPP_
#define GLOBAL_CONFIG_HPP_

#include <Arduino.h>

/************************ RTOS設定 ここから ************************/
#define ADT_STACk_SIZE      (1024)
#define VDT_STACk_SIZE      (1024)
#define FDT_STACk_SIZE      (512)
#define RMT_STACk_SIZE      (2048)
#define CGT_STACk_SIZE      (512)
#define DEBUG_STACk_SIZE    (512)
#define LGT_STACk_SIZE      (512)
#define IDLETASK_STACk_SIZE (128)

#define ADT_PRIORITY      (1)
#define VDT_PRIORITY      (2)
#define FDT_PRIORITY      (1)
#define RMT_PRIORITY      (2)
#define CGT_PRIORITY      (1)
#define DEBUG_PRIORITY    (0)
#define LGT_PRIORITY      (0)
#define IDLETASK_PRIORITY (0)

// MSG Bufferサイズ(Msg共用体サイズ何個分のバッファを用意するか)
#define ADT_MSG_REQ_BUFFER_SIZE (3)
#define VDT_MSG_REQ_BUFFER_SIZE (3)
#define CGT_MSG_REQ_BUFFER_SIZE (2)

#define ENABLE_FREERTOS_TASK_STACK_PRINT (0) // IdleタスクでRTOSスタックサイズ測定を行うかどうか
/************************ RTOS設定 ここまで ************************/

/************************ Network設定 ここから ************************/
// #define USE_HOME_NETWORK

/************************ Network設定 ここまで ************************/

/************************ DEBUG PRINT設定 ここから ************************/
#include "../src/Debug/Debug_task_main.hpp"
#include "../src/Logger/Logger_task_main.hpp"
template <typename... Args>
void debug_printf(const char *format, Args const &...args) {
  uint16_t u16_print_size = sprintf((char *)DEBUG::EXT_PRINT_BUF, format, args...);
  if(u16_print_size >= 1024) u16_print_size = 1024;
  DEBUG::print(DEBUG::EXT_PRINT_BUF, u16_print_size);
  LGT::push_buffer(DEBUG::EXT_PRINT_BUF, u16_print_size);
}

//#define DEBUG_PRINT_ADT(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_ADT(fmt, ...)

//#define DEBUG_PRINT_STR_ADT(fmt) debug_printf(fmt)
#define DEBUG_PRINT_STR_ADT(fmt)

//#define DEBUG_PRINT_FDT(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_FDT(fmt, ...)

//#define DEBUG_PRINT_VDT(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT(fmt, ...)

//#define DEBUG_PRINT_STR_VDT(fmt) debug_printf(fmt)
#define DEBUG_PRINT_STR_VDT(fmt)

//#define DEBUG_PRINT_VDT_IMU(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT_IMU(fmt, ...)

//#define DEBUG_PRINT_VDT_MOTOR(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_VDT_MOTOR(fmt, ...)

//#define DEBUG_PRINT_RMT(fmt, ...) debug_printf(fmt, __VA_ARGS__)
#define DEBUG_PRINT_RMT(fmt, ...)

//#define DEBUG_PRINT_STR_RMT(fmt) debug_printf(fmt)
#define DEBUG_PRINT_STR_RMT(fmt, ...)
/************************ DEBUG PRINT設定 ここまで ************************/

/************************ DEBUG TASK負荷測定設定 ここから ************************/
#define ENABLE_PRINT_PROCESS_LOAD (0)

enum DBG_PRC_ID {
  ADT_MAIN = 0x10,
  ADT_CAN2,
  ADT_CAN3,

  VDT_MAIN = 0x20,
  VDT_CAN_RX,
  VDT_CAN_TX,

  FDT_MAIN = 0x30,
  RMT_MAIN = 0x40,
  CGT_MAIN = 0x50,
  LOG_MAIN = 0xE0,
  DBG_MAIN = 0xF0,
};

#if ENABLE_PRINT_PROCESS_LOAD
#define DEBUG_PRINT_PRC_START(proc_id)  DEBUG::record_proc_load(proc_id, 1)
#define DEBUG_PRINT_PRC_FINISH(proc_id) DEBUG::record_proc_load(proc_id, 0)
#else
#define DEBUG_PRINT_PRC_START(proc_id)
#define DEBUG_PRINT_PRC_FINISH(proc_id)
#endif

/************************ DEBUG TASK負荷測定設定 ここまで ************************/

#endif