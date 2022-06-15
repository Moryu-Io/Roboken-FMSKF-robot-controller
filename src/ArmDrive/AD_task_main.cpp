// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <HardwareSerial.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "AD_can_controller_gim.hpp"
#include "AD_can_controller_mybldc.hpp"
#include "AD_joint_gim_servo.hpp"
#include "AD_joint_ics_servo.hpp"
#include "AD_joint_mybldc_servo.hpp"
#include "AD_mode_base.hpp"
#include "AD_mode_initialize.hpp"
#include "AD_task_main.hpp"
#include "global_config.hpp"

namespace ADT {

// Peripheral設定
constexpr uint8_t U8_SERIAL_EN_PIN = 32;

// 通信管理
IcsHardSerialClass icsHardSerial(&Serial7, U8_SERIAL_EN_PIN, 115200, 10);
CAN_CTRL_MSV<CAN2> MSV_CAN;
CAN_CTRL_GIM<CAN3> GIM_CAN;

// 関節管理
JointBase::ConstParams j_Y0_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_curlim_default_A = 3.0f,
    .fl_mechend_pos_deg  = -45.0f,
    .fl_vel_init_degps   = 15.0f,
    .fl_curlim_init_A    = 1.0f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_P1_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_curlim_default_A = 3.0f,
    .fl_mechend_pos_deg  = 60.0f,
    .fl_vel_init_degps   = 30.0f,
    .fl_curlim_init_A    = 1.0f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_DF_Left_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = -45.0f,
    .fl_vel_init_degps   = 10.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_DF_Right_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = -45.0f,
    .fl_vel_init_degps   = 10.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_P3_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 3.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = 85.0f,
    .fl_vel_init_degps   = 45.0f,
    .fl_curlim_init_A    = 0.3f,
    .fl_initpos_deg      = 0.0f,
};
JointIcsServo    j_Y0(j_Y0_CParams);
JointGimServo    j_P1(j_P1_CParams);
JointMyBldcServo j_DF_Left(j_DF_Left_CParams, 1);   // 差動関節左
JointMyBldcServo j_DF_Right(j_DF_Right_CParams, 2); // 差動関節右
JointMyBldcServo j_P3(j_P3_CParams, 3);

// リンク
template <>
JointMyBldcServo *CAN_CTRL_MSV<CAN2>::p_servo_if[3] = {&j_DF_Left, &j_DF_Right, &j_P3};

template <>
JointGimServo *CAN_CTRL_GIM<CAN3>::p_servo_if = &j_P1;

JointBase *ADTModeBase::P_JOINT_[JointAxis::J_NUM] = {&j_Y0, &j_P1, &j_DF_Left, &j_P3};
float ADTModeBase::FL_CYCLE_TIME_S = 0.01f;

// Mode管理
ADTModeOff        m_off;
ADTModeInitialize m_init;

ADTModeBase *m_nowProcess  = &m_off; // 実行中のMode
ADTModeBase *m_nextProcess = &m_off; // 次回のMode

// テスト用
float   ang_array[2]   = {30.0f, -30.0f};
uint8_t ang_array_head = 0;

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  // Yaw0軸サーボ初期化
  pinMode(U8_SERIAL_EN_PIN, OUTPUT);
  icsHardSerial.begin();
  j_Y0.init(&icsHardSerial, 0);
  // j_Y0.set_torque_on(true);
  j_Y0.set_torque_on(false);
  j_P1.set_torque_on(true);
  j_P1.set_gain(4095, 100);

  // Pitch0軸サーボ初期化
  GIM_CAN.init();

  // 手首軸サーボ初期化(CAN)
  MSV_CAN.init();
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / 100;
  uint32_t counter = 0;

  //vTaskDelay(5000);
  //set_next_mode(INIT);

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    /* 命令確認(Mode遷移) */
    if((m_nowProcess != m_nextProcess) && m_nowProcess->isCompleted()) {
      m_nowProcess->end();
      m_nowProcess = m_nextProcess;
      m_nowProcess->init();
    }

    /* 命令確認(Modeへの指示) */

    /* Mode処理 */
    m_nowProcess->update();

    /* CAN系を先に通信する */
    j_P1.update();
    j_P3.update();
    GIM_CAN.tx_routine();
    MSV_CAN.tx_routine();

    /* UART系 */
    j_Y0.update();

    /* 公開情報の関節の状態を更新する */

    /* デバッグ */
    if(counter > 100){
      DEBUG_PRINT_ADT("[ADT]%d,%d\n", (int)j_P1.get_now_deg(), (int)j_P3.get_now_deg());
      counter = 0;
    }else{
      counter++;
    }
    // DEBUG_PRINT_ADT("[ADT]%d,%d\n", micros(), (int)j_Y0.get_now_deg());
    // 
  }
}

/**
 * @brief Set the next mode object
 * 
 * @param _id 
 */
void set_next_mode(MODE_ID _id){
  switch (_id)
  {
  case MODE_ID::OFF:
    m_nextProcess = &m_off;
    break;
  case MODE_ID::INIT:
    m_nextProcess = &m_init;
    break;
  default:
    break;
  }
}

}; // namespace ADT