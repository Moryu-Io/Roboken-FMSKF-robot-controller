// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <ADC.h>
#include <DMAChannel.h>

// ローカル
#include "../ArmDrive/AD_task_main.hpp"
#include "../FloorDetect/FD_task_main.hpp"
#include "../RobotManager/RM_task_main.hpp"
#include "../CameraGimbal/CG_task_main.hpp"
#include "../Utility/util_gptimer.hpp"
#include "../Utility/util_led.hpp"
#include "../VehicleDrive/VD_task_main.hpp"
#include "Debug_task_main.hpp"
#include "global_config.hpp"

#if ENABLE_FREERTOS_TASK_STACK_PRINT
extern TaskHandle_t ArmDriveTask_handle;
extern TaskHandle_t VehicleDriveTask_handle;
extern TaskHandle_t FloorDetectTask_handle;
extern TaskHandle_t RobotManagerTask_handle;
extern TaskHandle_t CameraGimbalTask_handle;
extern TaskHandle_t DebugTask_handle;
extern TaskHandle_t IdleTask_handle;
#endif

namespace DEBUG {

#define DEBUGPRINT_BUFLEN (4096)
#define PROCLOAD_BUFLEN   (4096*4)

template <int buflen>
struct PrintBuffer {
  uint8_t  u8_buf_[buflen];
  uint32_t u32_head_;
};

PrintBuffer<DEBUGPRINT_BUFLEN> DEBUG_PRINT_BUF[2];
uint8_t                        U8_WRITE_PAGE = 0;

// 外部でSprintfするためのBuffer
char EXT_PRINT_BUF[1024];

// 処理負荷計測用
constexpr uint8_t            U8_PROCLOAD_DATA_LEN = 6;
PrintBuffer<PROCLOAD_BUFLEN> PROCLOAD_BUF[2];
uint8_t                      U8_PL_WRITE_PAGE = 0;
bool                         IS_SATRT_RECORD  = false;

// RTOS debug variable
char CR_RTOS_RUNTIME_STATUS_BUF[512] = {};
bool IS_RTOS_RUNTIME_MEASURED        = false;

uint32_t       U32_RTOS_RUNTIME_MEAS_MS_CNT = 0;
const uint32_t CU32_RTOS_RUNTIME_MEAS_MS    = 10000;

// Vehicle Drive Debug
bool IS_VDT_DRV_DEBUG_MODE = false;

void process_inputchar();
bool subproc_vehicle_drive();

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / 60;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(DBG_PRC_ID::DBG_MAIN);

    /* Debug処理 */
    if(IS_RTOS_RUNTIME_MEASURED && ((millis() - U32_RTOS_RUNTIME_MEAS_MS_CNT) > CU32_RTOS_RUNTIME_MEAS_MS)) {
      IS_RTOS_RUNTIME_MEASURED = false;
#if configGENERATE_RUN_TIME_STATS
      vTaskGetRunTimeStats(CR_RTOS_RUNTIME_STATUS_BUF);
      Serial.printf("%s", CR_RTOS_RUNTIME_STATUS_BUF);
#endif
      stop_gptimer_cnt();
    }

    /* 入力処理 */
    if(IS_VDT_DRV_DEBUG_MODE){
      subproc_vehicle_drive();
    } else {
      process_inputchar();
    }

    /* 出力処理 */
    /* 書き込みページ切り替え */
    uint8_t _u8_read_page = U8_WRITE_PAGE;
    U8_WRITE_PAGE         = U8_WRITE_PAGE ^ 1;

    if(DEBUG_PRINT_BUF[_u8_read_page].u32_head_ != 0) {
      /* 書き込みデータがある場合 */
      Serial.write(DEBUG_PRINT_BUF[_u8_read_page].u8_buf_, DEBUG_PRINT_BUF[_u8_read_page].u32_head_);
      DEBUG_PRINT_BUF[_u8_read_page].u32_head_ = 0;
    }

    if(IS_SATRT_RECORD) {
      uint8_t _u8_pl_read_page = U8_PL_WRITE_PAGE;
      U8_PL_WRITE_PAGE         = U8_PL_WRITE_PAGE ^ 1;

      if(PROCLOAD_BUF[_u8_pl_read_page].u32_head_ != 0) {
        /* 書き込みデータがある場合 */
        Serial.write(PROCLOAD_BUF[_u8_pl_read_page].u8_buf_, PROCLOAD_BUF[_u8_pl_read_page].u32_head_);
        PROCLOAD_BUF[_u8_pl_read_page].u32_head_ = 0;
      }
    }

    
    DEBUG_PRINT_PRC_FINISH(DBG_PRC_ID::DBG_MAIN);
  }
}

/**
 * @brief
 *
 * @param _buf
 * @param _size
 */
void print(char *_buf, uint32_t _size) {
  uint32_t _u32_head     = DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_;
  uint32_t _u32_rest_len = DEBUGPRINT_BUFLEN - _u32_head;
  if(_u32_rest_len >= _size) {
    /* バッファ残りより少ない量の書き込みの場合 */
    memcpy(&DEBUG_PRINT_BUF[U8_WRITE_PAGE].u8_buf_[_u32_head], _buf, _size);
    DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_ += _size;
  } else {
    /* バッファ残りより多い場合、書き込める場所まで書き込む */
    memcpy(&DEBUG_PRINT_BUF[U8_WRITE_PAGE].u8_buf_[_u32_head], _buf, _u32_rest_len);
    DEBUG_PRINT_BUF[U8_WRITE_PAGE].u32_head_ += _u32_rest_len;
  }
}

/**
 * @brief
 *
 * @param _buf
 * @param _size
 */
void record_proc_load(uint8_t prc_id, uint8_t is_start) {
  if(!IS_SATRT_RECORD) return;

  uint32_t _u32_head      = PROCLOAD_BUF[U8_PL_WRITE_PAGE].u32_head_;
  uint32_t _u32_now_count = get_debug_cnt();
  uint32_t _u32_rest_len  = PROCLOAD_BUFLEN - _u32_head;
  if(_u32_rest_len >= U8_PROCLOAD_DATA_LEN) {
    /* バッファ残りより少ない量の書き込みの場合 */
    // 先にheadを進めておく（割り込み対策）
    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u32_head_ += U8_PROCLOAD_DATA_LEN;

    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u8_buf_[_u32_head]     = prc_id;
    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u8_buf_[_u32_head + 1] = is_start;
    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u8_buf_[_u32_head + 2] = _u32_now_count & 0x000000FF;
    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u8_buf_[_u32_head + 3] = (uint8_t)((_u32_now_count & 0x0000FF00) >> 8);
    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u8_buf_[_u32_head + 4] = (uint8_t)((_u32_now_count & 0x00FF0000) >> 16);
    PROCLOAD_BUF[U8_PL_WRITE_PAGE].u8_buf_[_u32_head + 5] = (uint8_t)((_u32_now_count & 0xFF000000) >> 24);

  } else {
    /* バッファ残りより多い場合、無効 */
  }
}

/**
 * @brief ArmDriveTask向けメニュー
 *
 */
static void subproc_adt_menu() {
  Serial.printf("[DEBUG]ADT MENU\n");
  Serial.printf("[DEBUG]f:OFF, i:init, p:positioning, t:posseq\n");
  while(Serial.available() < 1) {};
  char _c = Serial.read();

  ADT::MSG_REQ adt_msg;

  switch(_c) {
  case 'i':
    /* 初期化モードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::INIT;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  case 'j':
    /* 初期位置移動モードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::INIT_POS_MOVE;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  case 'f':
    /* OFFモードへの遷移指示(強制) */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::OFF;
    adt_msg.change_mode.u8_forced   = 1;
    ADT::send_req_msg(&adt_msg);
    break;
  case 'p':
    /* Positioningモードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::POSITIONING;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  case 't':
    /* PositioningSeqモードへの遷移指示 */
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::POSITIONING_SEQ;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  case 'd':
    /* PositioningSeqへのdebug駆動指示 */
    adt_msg.common.MsgId          = ADT::MSG_ID::REQ_DBG_TIMEANGLE;
    adt_msg.dbg_time_angle.u32_id = 0;
    ADT::send_req_msg(&adt_msg);
    break;
  default:
    break;
  }
};

/**
 * @brief VehicleDriveTask向けメニュー
 *
 */
static void subproc_vdt_menu() {
  Serial.printf("[DEBUG]VDT MENU\n");
  Serial.printf("[DEBUG]s:stop, hjklyu:dir\n");
  IS_VDT_DRV_DEBUG_MODE = true;
};

bool subproc_vehicle_drive(){
  if(Serial.available() < 1) return false;
  char _c = Serial.read();

  VDT::MSG_REQ vdt_msg;

  switch(_c) {
  case 's':
    /* STOP指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 0;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'h':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_LEFT;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'j':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_BACK;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'k':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_FORWARD;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'l':
    /* ← 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_RIGHT;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'y':
    /* ←回転 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::ROT_LEFT;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  case 'u':
    /* →回転 指示 */
    vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
    vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::ROT_RIGHT;
    vdt_msg.move_dir.u32_speed   = 0;
    vdt_msg.move_dir.u32_time_ms = 100;
    VDT::send_req_msg(&vdt_msg);
    break;
  default:
    IS_VDT_DRV_DEBUG_MODE = false;
    break;
  }

  return true;
}

static void subproc_cgt_menu(){
  Serial.printf("[DEBUG]CGT MENU\n");
  Serial.printf("[DEBUG]i:init,u:up,d:down\n");
  while(Serial.available() < 1) {};
  char _c = Serial.read();

  CGT::MSG_REQ cgt_msg;

  switch(_c) {
  case 'i':
    /* カメラジンバル初期化指示 */
    cgt_msg.common.MsgId = CGT::MSG_ID::REQ_INIT;
    CGT::send_req_msg(&cgt_msg);
    break;
  case 'u':
    /* 上向き */
    cgt_msg.common.MsgId = CGT::MSG_ID::REQ_MOVE_PITCH;
    cgt_msg.move_pitch.fl_pitch_deg = 30.0f;
    CGT::send_req_msg(&cgt_msg);
    break;
  case 'd':
    /* 下向き */
    cgt_msg.common.MsgId = CGT::MSG_ID::REQ_MOVE_PITCH;
    cgt_msg.move_pitch.fl_pitch_deg = -30.0f;
    CGT::send_req_msg(&cgt_msg);
    break;
  default:
    break;
  }
}


/**
 *
 */
static void subproc_debug_menu() {
  Serial.printf("[DEBUG]DEBUG MENU\n");
  Serial.printf("[DEBUG]r:rtos proctime, s:stacksize, p:pl start, f:pl finish\n");
  while(Serial.available() < 1) {};
  char _c = Serial.read();

  switch(_c) {
  case 'r':
    /* RTOS処理時間測定開始 */
    IS_RTOS_RUNTIME_MEASURED     = true;
    U32_RTOS_RUNTIME_MEAS_MS_CNT = millis();
    start_gptimer_cnt();
    break;
  case 's':
    /* stack size */
#if ENABLE_FREERTOS_TASK_STACK_PRINT
  {
    int ADT_max_stack_size      = ADT_STACk_SIZE - uxTaskGetStackHighWaterMark(ArmDriveTask_handle);
    int VDT_max_stack_size      = VDT_STACk_SIZE - uxTaskGetStackHighWaterMark(VehicleDriveTask_handle);
    int FDT_max_stack_size      = FDT_STACk_SIZE - uxTaskGetStackHighWaterMark(FloorDetectTask_handle);
    int RMT_max_stack_size      = RMT_STACk_SIZE - uxTaskGetStackHighWaterMark(RobotManagerTask_handle);
    int CGT_max_stack_size      = CGT_STACk_SIZE - uxTaskGetStackHighWaterMark(CameraGimbalTask_handle);
    int IdleTask_max_stack_size = IDLETASK_STACk_SIZE - uxTaskGetStackHighWaterMark(IdleTask_handle);

    Serial.printf("[StackSize]ADT:%d,VDT:%d,FDT:%d,RMT:%d,CGT:%d,Idl:%d\n", ADT_max_stack_size,
                  VDT_max_stack_size,
                  FDT_max_stack_size,
                  RMT_max_stack_size,
                  CGT_max_stack_size,
                  IdleTask_max_stack_size);
  }
#endif
    break;
  case 'p':
    /* 全処理時間測定開始 */
    start_debug_cnt();
    IS_SATRT_RECORD = true;
    break;
  case 'f':
    /* 全処理時間測定終了 */
    IS_SATRT_RECORD = false;
    stop_debug_cnt();
    break;
  default:
    break;
  }
};

/**
 * @brief シリアル入力処理
 *
 */
void process_inputchar() {
  if(Serial.available()) {
    char _c = Serial.read();
    switch(_c) {
    case 'a':
      subproc_adt_menu();
      break;
    case 'v':
      subproc_vdt_menu();
      break;
    case 'c':
      subproc_cgt_menu();
      break;
    case 't':
      subproc_debug_menu();
      break;
    default:
      break;
    }
  }
}

}; // namespace DEBUG
