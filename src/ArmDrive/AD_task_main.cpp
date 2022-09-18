// RTOS
#include <FreeRTOS_TEENSY4.h>
#include <message_buffer.h>

// Arduinoライブラリ
#include <HardwareSerial.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "AD_can_controller_gim.hpp"
#include "AD_can_controller_mg.hpp"
#include "AD_can_controller_mybldc.hpp"
#include "AD_joint_dfgear.hpp"
#include "AD_joint_gim_servo.hpp"
#include "AD_joint_ics_servo.hpp"
#include "AD_joint_mybldc_servo.hpp"
#include "AD_mode_base.hpp"
#include "AD_mode_initialize.hpp"
#include "AD_mode_initpos_move.hpp"
#include "AD_mode_positioning.hpp"
#include "AD_mode_positioning_seq.hpp"
#include "AD_mode_positioning_seq_debug_data.hpp"
#include "AD_task_main.hpp"
#include "global_config.hpp"

namespace ADT {

// Peripheral設定
constexpr uint8_t U8_SERIAL_EN_PIN = 32;

// 通信管理
IcsHardSerialClass icsHardSerial(&Serial7, U8_SERIAL_EN_PIN, 115200, 10);
CAN_CTRL_MSV<CAN2> MSV_CAN;
//CAN_CTRL_GIM<CAN3> GIM_CAN;
CAN_CTRL_MG<CAN3> MG_CAN;

// 関節管理
JointBase::ConstParams j_Y0_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 3.0f,
    .fl_mechend_pos_deg  = -45.0f,
    .fl_vel_init_degps   = 15.0f,
    .fl_curlim_init_A    = 1.0f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_P1_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 4.0f,
    .fl_mechend_pos_deg  = 150.0f,
    .fl_vel_init_degps   = 30.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = 120.0f,
};
JointBase::ConstParams j_DF_Left_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = 0.0f,
    .fl_vel_init_degps   = 10.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_DF_Right_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = 0.0f,
    .fl_vel_init_degps   = 10.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_DF_Pt_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 24.0f / 7.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = 0.0f,
    .fl_vel_init_degps   = 30.0f,
    .fl_curlim_init_A    = 1.0f,
    .fl_initpos_deg      = -60.0f,
};
JointBase::ConstParams j_DF_Rl_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 48.0f / 7.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 1.0f,
    .fl_mechend_pos_deg  = 0.0f,
    .fl_vel_init_degps   = 30.0f,
    .fl_curlim_init_A    = 1.0f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_P3_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 48.0f / 19.0f,
    .fl_motor_dir        = -1.0f,
    .fl_curlim_default_A = 0.8f,
    .fl_mechend_pos_deg  = -90.0f,
    .fl_vel_init_degps   = -60.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = -10.0f,
};
JointIcsServo      j_Y0(j_Y0_CParams);
//JointGimServo      j_P1(j_P1_CParams);
JointMgServo      j_P1(j_P1_CParams);
JointMyBldcServo   j_DF_Left(j_DF_Left_CParams, 1);     // 差動関節左
JointMyBldcServo   j_DF_Right(j_DF_Right_CParams, 2);   // 差動関節右
JointDfGearVirtual j_DF_Virtual(j_DF_Left, j_DF_Right); // 差動関節両軸管理
JointDfGearPitch   j_P2(j_DF_Pt_CParams, j_DF_Virtual); // 差動関節Pitch
JointDfGearRoll    j_R0(j_DF_Rl_CParams, j_DF_Virtual); // 差動関節Pitch
JointMyBldcServo   j_P3(j_P3_CParams, 3);

// リンク
template <>
JointMyBldcServo *CAN_CTRL_MSV<CAN2>::p_servo_if[3] = {&j_DF_Left, &j_DF_Right, &j_P3};

template <>
JointMgServo *CAN_CTRL_MG<CAN3>::p_servo_if = &j_P1;
template <>
uint8_t CAN_CTRL_MG<CAN3>::u8_now_push_buf = 0;
template <>
void CAN_CTRL_MG<CAN3>::rxmb_callback(const CAN_message_t &msg){
  /* 内部でMG_CANの関数を呼ぶ必要があるため暫定でここで関数宣言している */
  if(p_servo_if == NULL) return;
  p_servo_if->rx_callback((JointMgServo::MgMsgRx *)msg.buf, (int16_t)(micros() & 0x7FFF));

  if(u8_now_push_buf == 1){
    /* MGは2連続CAN送信を受け付けてくれないので、返答があってから2個目を投げる */
    CAN_message_t tx_msg;
    tx_msg.id = 0x141;
    if(p_servo_if->get_cantx2_data(tx_msg.buf)){
      /* 送信データがある場合 */
      MG_CAN.write((FLEXCAN_MAILBOX)(NUM_RX_MAILBOXES_GIMMG_CAN+2), tx_msg);
      u8_now_push_buf = 2;
    }
  }
};

JointBase *ADTModeBase::P_JOINT_[JointAxis::J_NUM] = {&j_Y0, &j_P1, &j_P2, &j_R0, &j_P3};
float      ADTModeBase::FL_CYCLE_TIME_S            = 0.01f;

// Mode管理
ADTModeOff            m_off;
ADTModeInitialize     m_init;
ADTModeInitPosMove    m_init_posmove; // 角度リセットはしない
ADTModePositioning    m_posi;
ADTModePositioningSeq m_posseq;

ADTModeBase *m_nowProcess  = &m_off; // 実行中のMode
ADTModeBase *m_nextProcess = &m_off; // 次回のMode

// RTOS メッセージ
MessageBufferHandle_t p_MsgBufReq;
MSG_REQ               msgReq;

static void process_message();
static void set_next_mode(MODE_ID _id, bool force);
static void set_move_cmd(MSG_ReqMovePos *_req);
static void set_timeangle_cmd(MSG_ReqMoveTimeAngle *_req);

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  p_MsgBufReq = xMessageBufferCreate(VDT_MSG_REQ_BUFFER_SIZE * sizeof(MSG_REQ));

  // Yaw0軸サーボ初期化
  pinMode(U8_SERIAL_EN_PIN, OUTPUT);
  icsHardSerial.begin();
  j_Y0.doinit(&icsHardSerial, 0);
  // j_Y0.set_torque_on(true);
  j_Y0.set_torque_on(false);

  j_P1.set_torque_on(false);
  j_P1.init();

  // Pitch0軸サーボ初期化
  MG_CAN.init();

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
  uint32_t counter   = 0;

  // vTaskDelay(5000);
  // set_next_mode(INIT);

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    /* Message処理 */
    process_message();

    /* Mode処理 */
    m_nowProcess->update();

    /* CAN系を先に通信する */
    j_P1.update();
    j_DF_Left.update();
    j_DF_Right.update();
    j_P3.update();
    MG_CAN.tx_routine();
    MSV_CAN.tx_routine();

    // GIM_CAN.events();
    // MSV_CAN.events();

    /* UART系 */
    j_Y0.update();

    /* 公開情報の関節の状態を更新する */

    /* デバッグ */
    if(counter > 1) {
      // DEBUG_PRINT_ADT("[ADT]%d,%d\n", (int)j_P3.get_now_deg(), (int)(j_P3.get_now_cur() * 100.0f));
      // debug_printf("[ADT]Tgt:%d,%d,%d,%d,%d\n", (int)j_Y0.get_tgt_deg(), (int)j_P1.get_tgt_deg(), (int)j_P2.get_tgt_deg(), (int)j_R0.get_tgt_deg(), (int)j_P3.get_tgt_deg());
      DEBUG_PRINT_ADT("[ADT]Pos:%d,%d,%d,%d,%d\n", (int)j_Y0.get_now_deg(), (int)j_P1.get_now_deg(), (int)j_DF_Left.get_now_deg(), (int)j_DF_Right.get_now_deg(), (int)j_P3.get_now_deg());
      // DEBUG_PRINT_ADT("[ADT]Pos:%d,%d,%d,%d\n", (int)j_DF_Left.get_now_deg(), (int)j_DF_Right.get_now_deg(), (int)j_P2.get_now_deg(), (int)j_R0.get_now_deg());

      counter = 0;
    } else {
      counter++;
    }
    // DEBUG_PRINT_ADT("[ADT]%d,%d\n", micros(), (int)j_Y0.get_now_deg());
    //
  }
}

/**
 * @brief
 *
 */
static void process_message() {
  if(xMessageBufferReceive(p_MsgBufReq, (void *)&msgReq, sizeof(MSG_REQ), 0) == sizeof(MSG_REQ)) {
    switch(msgReq.common.MsgId) {
    case MSG_ID::REQ_CHANGE_MODE:
      /* MODE変更指示 */
      set_next_mode((MODE_ID)msgReq.change_mode.u32_mode_id, (bool)msgReq.change_mode.u8_forced);
      break;
    case MSG_ID::REQ_MOVE_POS:
      /* POS変更指示 */
      set_move_cmd(&msgReq.move_pos);
      break;
    case MSG_ID::REQ_MOVE_TIMEANGLE:
      /* POS変更指示 */
      set_timeangle_cmd(&msgReq.time_angle);
      break;
    case MSG_ID::REQ_DBG_TIMEANGLE:
      /* TimeAngleDebug動作 */
      /* 現MODEがPOSITIONING_SEQで無い場合は破棄 */
      if(m_nowProcess != &m_posseq) {
        return;
      }
      m_posseq.push_cmdseq(*get_poscmdseq_debug());
    default:
      break;
    }
  }
}

/**
 * @brief Set the next mode object
 *
 * @param _id
 */
static void set_next_mode(MODE_ID _id, bool force) {
  switch(_id) {
  case MODE_ID::OFF:
    DEBUG_PRINT_STR_ADT("[ADT]MODE:OFF\n");
    m_nextProcess = &m_off;
    break;
  case MODE_ID::INIT:
    DEBUG_PRINT_STR_ADT("[ADT]MODE:INIT\n");
    m_nextProcess = &m_init;
    break;
  case MODE_ID::INIT_POS_MOVE:
    DEBUG_PRINT_STR_ADT("[ADT]MODE:INITPOS_MOVE\n");
    m_nextProcess = &m_init_posmove;
    break;
  case MODE_ID::POSITIONING:
    DEBUG_PRINT_STR_ADT("[ADT]MODE:POSITIONING\n");
    m_nextProcess = &m_posi;
    break;
  case MODE_ID::POSITIONING_SEQ:
    DEBUG_PRINT_STR_ADT("[ADT]MODE:POSITIONING_SEQ\n");
    m_nextProcess = &m_posseq;
    break;
  default:
    break;
  }

  /* 違うモードが設定されており、現モードが完了or強制終了フラグがTrueの場合 */
  if((m_nowProcess != m_nextProcess) && (m_nowProcess->isCompleted() || force)) {
    m_nowProcess->end();
    m_nowProcess = m_nextProcess;
    m_nowProcess->init();
  }
}

/**
 * @brief Set the move cmd object
 *
 * @param _req
 */
static void set_move_cmd(MSG_ReqMovePos *_req) {
  /* 現MODEがPOSITIONINGで無い場合は破棄 */
  if(m_nowProcess != &m_posi) {
    return;
  }

  ADTModePositioning::PosCmd _cmd = {};

  _cmd.u32_id    = _req->u32_id;
  _cmd.u32_dt_ms = _req->u32_dt_ms;
  for(int i = 0; i < 5; i++) _cmd.fl_tgt_pos_deg[i] = _req->fl_pos[i];

  m_posi.push_cmd(_cmd);
}

/**
 * @brief Set the move cmd object
 *
 * @param _req
 */
static void set_timeangle_cmd(MSG_ReqMoveTimeAngle *_req) {
  /* 現MODEがPOSITIONING_SEQで無い場合は破棄 */
  /* [暫定]現MODEがINIT関連の場合も受け付けておく */
  //if((m_nowProcess != &m_posseq) && (m_nowProcess != &m_init) && (m_nowProcess != &m_init_posmove)) {
  //  return;
  //}

  ADTModePositioningSeq::PosCmdSeq _cmdseq = {};

  _cmdseq.u32_id         = _req->u32_id;
  _cmdseq.u8_cmd_seq_len = _req->u32_len;

  for(int i = 0; i < (int)_req->u32_len; i++) {
    _cmdseq.cmd_seq[i].u32_dt_ms         = _req->ptr_tAng->arm[0].point.data[i].dt;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[0] = _req->ptr_tAng->arm[0].point.data[i].theta * 57.29578f; // rad to deg
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[1] = _req->ptr_tAng->arm[1].point.data[i].theta * 57.29578f;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[2] = _req->ptr_tAng->arm[2].point.data[i].theta * 57.29578f;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[3] = _req->ptr_tAng->arm[3].point.data[i].theta * 57.29578f;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[4] = _req->ptr_tAng->arm[4].point.data[i].theta * 57.29578f;
  }

  m_posseq.push_cmdseq(_cmdseq);
}

/**
 * @brief
 *
 * @param _msg
 */
void send_req_msg(MSG_REQ *_msg) {
  xMessageBufferSend(p_MsgBufReq, (void *)_msg, sizeof(MSG_REQ), 0);
}

/**
 * @brief Positioningコマンドの処理状況を問い合わせ(即時回答)
 *
 * @param _cmdid
 * @return uint32_t
 */
uint32_t get_status_movepos_proc(uint32_t _cmdid) {
  return m_posi.get_q_cmd_status(_cmdid);
}

/**
 * @brief PositioningSeqコマンドの処理状況を問い合わせ(即時回答)
 *
 * @param _cmdid
 * @return uint32_t
 */
uint32_t get_status_timeangle_proc(uint32_t _cmdid) {
  return m_posseq.get_q_cmdseq_status(_cmdid);
}

}; // namespace ADT