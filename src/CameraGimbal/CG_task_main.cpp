// RTOS
#include <FreeRTOS_TEENSY4.h>
#include <message_buffer.h>

// Arduinoライブラリ
#include <HardwareSerial.h>

// ローカル
#include "CG_ics_servo.hpp"
#include "CG_task_main.hpp"
#include "IcsHardSerialClass.h"
#include "global_config.hpp"

namespace CGT {

// ローカルパラメータ定義
constexpr uint32_t U32_CG_TASK_CTRL_FREQ_HZ = 30;
constexpr uint8_t  U8_CAM_PITCH_SERVO_ID    = 0;
constexpr float    FL_CAM_PITCH_DEG_DEFAULT = -15.8f;
constexpr uint8_t  U8_CAM_YAW_SERVO_ID      = 1;
constexpr float    FL_CAM_YAW_DEG_DEFAULT   = 0;
CGIcsServo         Cam_Picth;
CGIcsServo         Cam_Yaw;

constexpr float    FL_CAM_PITCH_DEG_LIM[2] = {-16.0f, -15.0f};
constexpr float    FL_CAM_YAW_DEG_LIM[2]   = {-20.0f,  20.0f};

// Peripheral設定
constexpr uint8_t U8_SERIAL_EN_PIN = 6;

// 通信管理
IcsHardSerialClass icsHardSerial(&Serial2, U8_SERIAL_EN_PIN, 115200, 10);

// RTOS メッセージ
MessageBufferHandle_t p_MsgBufReq;
MSG_REQ               msgReq;

static void process_message();
static void job_init();
static void job_move_pitch(float _pitchdeg);
static void job_default_pitch();
static void job_move_yaw(float _yawdeg);
static void job_default_yaw();

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  p_MsgBufReq = xMessageBufferCreate(CGT_MSG_REQ_BUFFER_SIZE * sizeof(MSG_REQ));

  pinMode(U8_SERIAL_EN_PIN, OUTPUT);
  icsHardSerial.begin();
  Cam_Picth.init(&icsHardSerial, U8_CAM_PITCH_SERVO_ID);
  Cam_Yaw.init(&icsHardSerial, U8_CAM_YAW_SERVO_ID);
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / U32_CG_TASK_CTRL_FREQ_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(CGT_MAIN);

    /* Message処理 */
    process_message();

    /* Servo更新 */
    Cam_Picth.update();
    Cam_Yaw.update();

    DEBUG_PRINT_PRC_FINISH(CGT_MAIN);
  }
}

void send_req_msg(MSG_REQ *_msg) {
  xMessageBufferSend(p_MsgBufReq, (void *)_msg, sizeof(MSG_REQ), 0);
}

static void process_message() {
  if(xMessageBufferReceive(p_MsgBufReq, (void *)&msgReq, sizeof(MSG_REQ), 0) == sizeof(MSG_REQ)) {
    switch(msgReq.common.MsgId) {
    case MSG_ID::REQ_INIT:
      /* INIT指示 */
      job_init();
      break;
    case MSG_ID::REQ_MOVE_PITCH:
      /* Pitch角度変更指示 */
      job_move_pitch(msgReq.move_pitch.fl_pitch_deg);
      break;
    case MSG_ID::REQ_DEFAULT_PITCH:
      /* Pitch角度をデフォルト位置へ */
      job_default_pitch();
      break;
    case MSG_ID::REQ_MOVE_YAW:
      /* Yaw角度変更指示 */
      job_move_yaw(msgReq.move_pitch.fl_pitch_deg);
      break;
    case MSG_ID::REQ_DEFAULT_YAW:
      /* Yaw角度をデフォルト位置へ */
      job_default_yaw();
      break;
    case MSG_ID::REQ_MOVE_PY:
      /* PY角度変更指示 */
      job_move_pitch(msgReq.move_py.fl_pitch_deg);
      job_move_yaw(msgReq.move_py.fl_yaw_deg);
      break;
    default:
      break;
    }
  }
}

static void job_init() {
  Cam_Picth.init(&icsHardSerial, U8_CAM_PITCH_SERVO_ID);
  Cam_Picth.set_torque(true);
  Cam_Picth.set_target_deg(FL_CAM_PITCH_DEG_DEFAULT);

  Cam_Yaw.init(&icsHardSerial, U8_CAM_YAW_SERVO_ID);
  Cam_Yaw.set_torque(true);
  Cam_Yaw.set_target_deg(FL_CAM_YAW_DEG_DEFAULT);
}

static void job_move_pitch(float _pitchdeg) {
  float tgt_deg =  (_pitchdeg > FL_CAM_PITCH_DEG_LIM[1]) ? FL_CAM_PITCH_DEG_LIM[1]
                 :((_pitchdeg < FL_CAM_PITCH_DEG_LIM[0]) ? FL_CAM_PITCH_DEG_LIM[0]
                                                         : _pitchdeg);
  Cam_Picth.set_target_deg(tgt_deg);
}

static void job_default_pitch() {
  Cam_Picth.set_target_deg(FL_CAM_PITCH_DEG_DEFAULT);
}

static void job_move_yaw(float _yawdeg) {
  float tgt_deg =  (_yawdeg > FL_CAM_YAW_DEG_LIM[1]) ? FL_CAM_YAW_DEG_LIM[1]
                 :((_yawdeg < FL_CAM_YAW_DEG_LIM[0]) ? FL_CAM_YAW_DEG_LIM[0]
                                                         : _yawdeg);
  Cam_Yaw.set_target_deg(tgt_deg);
}

static void job_default_yaw() {
  Cam_Yaw.set_target_deg(FL_CAM_YAW_DEG_DEFAULT);
}

float get_pitch_angle_deg() {
  return Cam_Picth.get_now_angle_deg();
}

float get_yaw_angle_deg() {
  return Cam_Yaw.get_now_angle_deg();
}

}; // namespace CGT
