// RTOS
#include <FreeRTOS_TEENSY4.h>
#include <message_buffer.h>

// Arduinoライブラリ
#include <TsyDMASPI.h>
#include <math.h>

// ローカル
#include "VD_can_controller.hpp"
#include "VD_imu_if_mpu6500.hpp"
#include "VD_motor_if_m2006.hpp"
#include "VD_task_main.hpp"
#include "VD_vehicle_controller.hpp"
#include "global_config.hpp"

namespace VDT {

// ローカルパラメータ定義
constexpr uint32_t U32_VEHICLE_CTRL_FREQ_HZ           = 1000;
constexpr uint32_t U32_VD_TASK_CTRL_FREQ_HZ           = 100;
constexpr float    FL_VEHICLE_DEFAULT_SPEED_MMPS      = 200;
constexpr float    FL_VEHICLE_DEFAULT_ROT_SPEED_RADPS = 2.0f * M_PI / 1.0f;
constexpr float    FL_VEHICLE_LIMIT_SPEED_MMPS        = 400;
constexpr float    FL_VEHICLE_LIMIT_ROT_SPEED_RADPS   = 6.0f * M_PI / 1.0f;

const Direction C_ACCEL_MAX_MOVE = {
    .x  = 1000.0f,
    .y  = 1000.0f,
    .th = 30.0f,
};
const Direction C_JERK_MAX_MOVE = {
    .x  = 10000.0f,
    .y  = 10000.0f,
    .th = 300.0f,
};
const Direction C_ACCEL_MAX_STOP = {
    .x  = 2000.0f,
    .y  = 2000.0f,
    .th = 70.0f,
};
const Direction C_JERK_MAX_STOP = {
    .x  = 30000.0f,
    .y  = 30000.0f,
    .th = 1000.0f,
};

// Peripheral設定
constexpr uint8_t U8_IMU1_CS_PIN = 37;
constexpr uint8_t U8_IMU2_CS_PIN = 36;

IntervalTimer canTxTimer; //タイマー割り込み

// デバイス設定
class IMU1 : public IMU_IF_MPU6500 {
public:
  void init() override {
    TsyDMASPI0.begin(U8_IMU1_CS_PIN, SPISettings(1000000, MSBFIRST, SPI_MODE3), true);
    LSBtoG     = 1.0f / 16834.0f;
    LSBtoRADPS = 3.141592f / (180.0f * 131.0f);
  }

protected:
  void kickSpiDma(uint8_t size) {
    TsyDMASPI0.queue(u8_tx_buf, u8_rx_buf, (size_t)size, U8_IMU1_CS_PIN);
  };
  bool isCompSpiDmaQueue() {
    return TsyDMASPI0.remained() == 0;
  };
};

// モータIF
MOTOR_IF_M2006 FL_motor(1);
MOTOR_IF_M2006 BL_motor(1);
MOTOR_IF_M2006 BR_motor(-1);
MOTOR_IF_M2006 FR_motor(-1);

// モータ用CAN IF
CAN_CTRL<CAN1> M_CAN;
template <>
MOTOR_IF_M2006 *CAN_CTRL<CAN1>::p_motor_if[4] = {&FL_motor, &BL_motor, &BR_motor, &FR_motor};

// モータ制御コントローラ
UTIL::FF_PI_D FL_m_ctrl((float)U32_VD_TASK_CTRL_FREQ_HZ, 0.005f, 0.0075f, 0.003f, 0.0f, 0.5f, 10.0f);
UTIL::FF_PI_D BL_m_ctrl((float)U32_VD_TASK_CTRL_FREQ_HZ, 0.005f, 0.0075f, 0.003f, 0.0f, 0.5f, 10.0f);
UTIL::FF_PI_D BR_m_ctrl((float)U32_VD_TASK_CTRL_FREQ_HZ, 0.005f, 0.0075f, 0.003f, 0.0f, 0.5f, 10.0f);
UTIL::FF_PI_D FR_m_ctrl((float)U32_VD_TASK_CTRL_FREQ_HZ, 0.005f, 0.0075f, 0.003f, 0.0f, 0.5f, 10.0f);

// IMU
static IMU1 imu1;

// 車体速度補間用
UTIL::VelInterpConstJerk VelIntpConstJerk_Xdir(1.0f / (float)U32_VEHICLE_CTRL_FREQ_HZ);
UTIL::VelInterpConstJerk VelIntpConstJerk_Ydir(1.0f / (float)U32_VEHICLE_CTRL_FREQ_HZ);
UTIL::VelInterpConstJerk VelIntpConstJerk_Tdir(1.0f / (float)U32_VEHICLE_CTRL_FREQ_HZ);

// Vehicleクラス用のパーツ
VEHICLE_CTRL::Parts vhcl_parts = {
    .p_imu        = &imu1,
    .p_vel_interp = {&VelIntpConstJerk_Xdir, &VelIntpConstJerk_Ydir, &VelIntpConstJerk_Tdir},
    .p_motor      = {&FL_motor, &BL_motor, &BR_motor, &FR_motor},
    .p_ctrl       = {&FL_m_ctrl, &BL_m_ctrl, &BR_m_ctrl, &FR_m_ctrl},
};

// インスタンス
static VEHICLE_CTRL vhclCtrl(vhcl_parts);

// RTOS メッセージ
MessageBufferHandle_t p_MsgBufReq;
MSG_REQ               msgReq;

void can_tx_routine_intr();

static float speed_limit(uint32_t u32_spd){
  if(u32_spd == 0) {
    return (float)FL_VEHICLE_DEFAULT_SPEED_MMPS;
  } else {
    return (float)((u32_spd > FL_VEHICLE_LIMIT_SPEED_MMPS) ? FL_VEHICLE_LIMIT_SPEED_MMPS : u32_spd);
  }
}

static float rot_speed_limit(uint32_t u32_spd){
  if(u32_spd == 0) {
    return (float)FL_VEHICLE_DEFAULT_ROT_SPEED_RADPS;
  } else {
    float fl_spd = (float)u32_spd * 0.1; // 10されて受け渡される
    return (float)((fl_spd > FL_VEHICLE_LIMIT_ROT_SPEED_RADPS) ? FL_VEHICLE_LIMIT_ROT_SPEED_RADPS : fl_spd);
  }
}


void prepare_task() {
  p_MsgBufReq = xMessageBufferCreate(VDT_MSG_REQ_BUFFER_SIZE * sizeof(MSG_REQ));

  FL_m_ctrl.set_FF_limit(1.0f);
  BL_m_ctrl.set_FF_limit(1.0f);
  BR_m_ctrl.set_FF_limit(1.0f);
  FR_m_ctrl.set_FF_limit(1.0f);

  imu1.init();
  M_CAN.init();

  canTxTimer.begin(can_tx_routine_intr, U32_VEHICLE_CTRL_FREQ_HZ);
}

void main(void *params) {
  uint32_t debug_counter = 0;
  uint32_t loop_tick     = (int)configTICK_RATE_HZ / U32_VD_TASK_CTRL_FREQ_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(VDT_MAIN);

    /* Msg処理 */
    if(xMessageBufferReceive(p_MsgBufReq, (void *)&msgReq, sizeof(MSG_REQ), 0) == sizeof(MSG_REQ)) {
      switch(msgReq.common.MsgId) {
      case REQ_MOVE_DIR: {
        DEBUG_PRINT_VDT("[VDT]MOVE_DIR:%d\n", msgReq.move_dir.u32_cmd);

        float speed = 0;

        /* 指示方向に応じて必要な要素を埋める */
        Direction move_dir = {};
        Direction accl_dir = {};
        Direction jerk_dir = {};
        switch(msgReq.move_dir.u32_cmd) {
        case GO_FORWARD:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = (float)speed;
          move_dir.y  = 0;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_BACK:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = -(float)speed;
          move_dir.y  = 0;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_RIGHT:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = 0;
          move_dir.y  = -(float)speed;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_LEFT:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = 0;
          move_dir.y  = (float)speed;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_RIGHT_FORWARD:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = (float)speed * sqrtf(2) * 0.5f;
          move_dir.y  = -(float)speed * sqrtf(2) * 0.5f;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_LEFT_FORWARD:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = (float)speed * sqrtf(2) * 0.5f;
          move_dir.y  = (float)speed * sqrtf(2) * 0.5f;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_RIGHT_BACK:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = -(float)speed * sqrtf(2) * 0.5f;
          move_dir.y  = -(float)speed * sqrtf(2) * 0.5f;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case GO_LEFT_BACK:
          speed = speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = -(float)speed * sqrtf(2) * 0.5f;
          move_dir.y  = (float)speed * sqrtf(2) * 0.5f;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case ROT_RIGHT:
          speed = rot_speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = 0;
          move_dir.y  = 0;
          move_dir.th = -speed;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case ROT_LEFT:
          speed = rot_speed_limit(msgReq.move_dir.u32_speed);
          move_dir.x  = 0;
          move_dir.y  = 0;
          move_dir.th = speed;
          accl_dir    = C_ACCEL_MAX_MOVE;
          jerk_dir    = C_JERK_MAX_MOVE;
          break;
        case MOVE_STOP:
        default:
          move_dir.x  = 0;
          move_dir.y  = 0;
          move_dir.th = 0;
          accl_dir    = C_ACCEL_MAX_STOP;
          jerk_dir    = C_JERK_MAX_STOP;
          break;
        }
        vhclCtrl.start();
        vhclCtrl.set_target_vel(move_dir, accl_dir, jerk_dir);
      } break;
      default:
        break;
      }
    }

    /* 車体制御Routine */
    // vhclCtrl.update();  // Timer割り込みで1kHz周期で計算する

    /* CAN Routine */
    // M_CAN.events(); // 多分不要
    // M_CAN.tx_routine();  // Timer割り込みで1kHz周期で送信する

    /* 以下、デバッグ用 */
    if(debug_counter == 0) {
      // Direction vhcl_pos;
      // vhclCtrl.get_vehicle_pos_mm_latest(vhcl_pos);
      //  DEBUG_PRINT_VDT_MOTOR("[VDT]%d,%d,%d\n", (int)vhcl_pos.x, (int)vhcl_pos.y, (int)vhcl_pos.th);
      Direction vhcl_vel;
      vhclCtrl.get_vehicle_vel_tgt_mmps_latest(vhcl_vel);
      // DEBUG_PRINT_VDT_MOTOR("[VDT]%d,%d,%d\n", (int)vhcl_vel.x, (int)vhcl_vel.y, (int)vhcl_vel.th);

      /* 車体モータデバッグ用 削除禁止 */
      int _fl_nowvel = FL_m_ctrl.get_now_val();
      int _bl_nowvel = BL_m_ctrl.get_now_val();
      int _br_nowvel = BR_m_ctrl.get_now_val();
      int _fr_nowvel = FR_m_ctrl.get_now_val();
      int _fl_tgtvel = FL_m_ctrl.get_target();
      int _bl_tgtvel = BL_m_ctrl.get_target();
      int _br_tgtvel = BR_m_ctrl.get_target();
      int _fr_tgtvel = FR_m_ctrl.get_target();
      DEBUG_PRINT_VDT_MOTOR("[VDT],%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", _fl_tgtvel, _fl_nowvel, FL_motor.get_rawCurr_tgt() //
                            ,
                            _bl_tgtvel, _bl_nowvel, BL_motor.get_rawCurr_tgt() //
                            ,
                            _br_tgtvel, _br_nowvel, BR_motor.get_rawCurr_tgt() //
                            ,
                            _fr_tgtvel, _fr_nowvel, FR_motor.get_rawCurr_tgt());
      debug_counter = 0;
    } else {
      debug_counter++;
    }

    DEBUG_PRINT_PRC_FINISH(VDT_MAIN);
  }
}

void can_tx_routine_intr() {
  DEBUG_PRINT_PRC_START(VDT_CAN_TX);
  vhclCtrl.update();
  M_CAN.tx_routine();
  DEBUG_PRINT_PRC_FINISH(VDT_CAN_TX);
}

void send_req_msg(MSG_REQ *_msg) {
  xMessageBufferSend(p_MsgBufReq, (void *)_msg, sizeof(MSG_REQ), 0);
}
}; // namespace VDT