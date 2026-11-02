// RTOS
#include <FreeRTOS_TEENSY4.h>
#include <message_buffer.h>
#include <queue.h>

// Arduinoライブラリ
#include <HardwareSerial.h>
#include <math.h>

// ローカル
#include "../Utility/util_iir.hpp"
#include "../Utility/util_led.hpp"
#include "../Utility/util_mymath.hpp"
#include "AD_can_controller_gim.hpp"
#include "AD_can_controller_mg.hpp"
#include "AD_can_controller_mybldc.hpp"
#include "AD_joint_dfgear.hpp"
#include "AD_joint_gim_servo.hpp"
#include "AD_joint_ics_servo.hpp"
#include "AD_joint_mybldc_servo.hpp"
#include "AD_kinematics.hpp"
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
    .fl_motor_dir        = -1.0f,
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
    .fl_curlim_default_A = 0.7f,
    .fl_mechend_pos_deg  = 150.0f,
    .fl_vel_init_degps   = 30.0f,
    .fl_curlim_init_A    = 0.15f,
    .fl_initpos_deg      = 145.0f,
};
JointBase::ConstParams j_DF_Left_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 0.5f,
    .fl_mechend_pos_deg  = 0.0f,
    .fl_vel_init_degps   = 10.0f,
    .fl_curlim_init_A    = 0.5f,
    .fl_initpos_deg      = 0.0f,
};
JointBase::ConstParams j_DF_Right_CParams = {
    .fl_ctrl_time_s      = 0.01f,
    .fl_gear_ratio       = 1.0f,
    .fl_motor_dir        = 1.0f,
    .fl_curlim_default_A = 0.5f,
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
    .fl_initpos_deg      = -90.0f,
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
    .fl_initpos_deg      = 0.0f,
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
  DEBUG_PRINT_PRC_START(ADT_CAN3);

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

  DEBUG_PRINT_PRC_FINISH(ADT_CAN3);
};

JointBase *ADTModeBase::P_JOINT_[JointAxis::J_NUM] = {&j_Y0, &j_P1, &j_P2, &j_R0, &j_P3};
float      ADTModeBase::FL_CYCLE_TIME_S            = 0.01f;

// 補正入力(RMTタスクが書き込み, ADTタスクが読み出し。単一float書き込みはCortex-M7でアトミック)
static volatile float    s_fl_yaw_corr_deg   = 0.0f; // 面向けYaw補正[deg]
static volatile uint32_t s_u32_yaw_corr_ms   = 0;    // Yaw補正の最終受信時刻[ms]
static volatile float    s_fl_height_err     = 0.0f; // 胴向け竹刀先端高さ誤差[m]
static volatile uint32_t s_u32_height_err_ms = 0;    // 高さ誤差の最終受信時刻[ms]

// ===== 面向けYaw補正(J0): トルクON中は全モードで常時適用 =====
static constexpr float    FL_YAW_CTRL_FREQ_HZ = 100.0f; // ADT制御周期[Hz]
static constexpr float    FL_YAW_CORR_MAX_DEG = 45.0f;  // Yaw補正のクランプ[deg]
static constexpr float    FL_YAW_SLEW_DEG     = 1.0f;   // 1周期あたりの補正変化量制限[deg/周期](=30deg/s@100Hz)
static constexpr uint32_t U32_YAW_STALE_MS    = 1000;    // この時間受信が無ければ補正を0へ戻す
static constexpr float    FL_YAW_LPF_CUTOFF   = 20.0f;  // Yaw補正LPFカットオフ(双一次)
static constexpr float    FL_YAW_LPF_A1 = (2.0f * FL_YAW_CTRL_FREQ_HZ - FL_YAW_LPF_CUTOFF) / (2.0f * FL_YAW_CTRL_FREQ_HZ + FL_YAW_LPF_CUTOFF);
static constexpr float    FL_YAW_LPF_B  = FL_YAW_LPF_CUTOFF / (2.0f * FL_YAW_CTRL_FREQ_HZ + FL_YAW_LPF_CUTOFF);

static UTIL::IIR1 yaw_corr_lpf(FL_YAW_LPF_A1, FL_YAW_LPF_B, FL_YAW_LPF_B);

// ===== 胴向けZ補正: PositioningSeq中に出力段オフセットとして適用 =====
static constexpr float    FL_Z_CTRL_FREQ_HZ = 100.0f;
static constexpr float    FL_Z_CLIK_GAIN = 0.20f;
static constexpr float    FL_Z_DLS_LAMBDA_MPRAD = 0.020f;
static constexpr float    FL_Z_CORR_MAX_RAD = 0.3490658503988659f; // 20deg
static constexpr float    FL_Z_SLEW_RAD = 0.008726646259971648f; // 0.5deg/cycle
static constexpr float    FL_RAD_PER_DEG = 0.01745329251994329577f;
static constexpr uint32_t U32_Z_STALE_MS = 100;
static constexpr float    FL_Z_LPF_CUTOFF = 10.0f;
static constexpr float    FL_Z_LPF_A1 = (2.0f * FL_Z_CTRL_FREQ_HZ - FL_Z_LPF_CUTOFF) / (2.0f * FL_Z_CTRL_FREQ_HZ + FL_Z_LPF_CUTOFF);
static constexpr float    FL_Z_LPF_B = FL_Z_LPF_CUTOFF / (2.0f * FL_Z_CTRL_FREQ_HZ + FL_Z_LPF_CUTOFF);

static UTIL::IIR1 z_err_lpf(FL_Z_LPF_A1, FL_Z_LPF_B, FL_Z_LPF_B);
static float      s_fl_z_corr_rad[JointAxis::J_NUM] = {};

// Mode管理
ADTModeOff            m_off;
ADTModeInitialize     m_init;
ADTModeInitPosMove    m_init_posmove; // 角度リセットはしない
ADTModePositioning    m_posi;
ADTModePositioningSeq m_posseq;

ADTModeBase *m_nowProcess  = &m_off; // 実行中のMode
ADTModeBase *m_nextProcess = &m_off; // 次回のMode

// RTOS メッセージ
QueueHandle_t         p_MsgQueueReq;
struct FollowTarget { float rad[5]; };
static QueueHandle_t p_FollowLatest;
static bool follow_armed = false;
static bool follow_permitted = false;
static volatile bool follow_locked = false;
static bool follow_active = false;
static uint32_t follow_received_ms = 0;
static FollowTarget follow_target = {};
static constexpr uint32_t FOLLOW_TIMEOUT_MS = 200;
// Servo bounds from EquationQuinque; Pitch1 includes the 145-degree KAMAE pose.
static constexpr float FOLLOW_MIN_RAD[5] = {-1.570796327f, 0.785398163f, -1.570796327f, -1.570796327f, -1.570796327f};
static constexpr float FOLLOW_MAX_RAD[5] = { 1.570796327f, 2.617993878f,  1.570796327f,  1.570796327f,  1.570796327f};
static constexpr float FOLLOW_LIMIT_EPS = 1e-4f;
MSG_REQ               msgReq;
static QueueHandle_t execution_events;
static bool emergency_stop_requested = false;
static int8_t manual_mode_request = -1;
static bool manual_enabled = false;
static volatile uint8_t manual_state_value = 0;
uint8_t manual_state() { return manual_state_value; }
static ExecutionProgress execution_progress = {};
static bool execution_progress_valid = false;

void report_execution_progress(uint32_t id, uint32_t elapsed_ms, uint8_t status) {
  const ExecutionProgress progress = {id, elapsed_ms, status};
  taskENTER_CRITICAL();
  execution_progress = progress;
  execution_progress_valid = true;
  taskEXIT_CRITICAL();
  if(status != 0) {
    // Terminal events survive the next sequence starting before RMT publishes.
    if(xQueueSend(execution_events, &progress, 0) != pdPASS) {
      DEBUG_PRINT_STR_ADT("[ADT]Execution event queue full\n");
    }
  }
}
bool take_execution_event(ExecutionProgress &progress) {
  return xQueueReceive(execution_events, &progress, 0) == pdPASS;
}
bool get_execution_progress(ExecutionProgress &progress) {
  taskENTER_CRITICAL();
  const bool valid = execution_progress_valid;
  progress = execution_progress;
  taskEXIT_CRITICAL();
  return valid;
}

static void process_message();
static void update_yaw_correction();
static void update_z_correction();
static void clear_z_correction();
static void release_z_correction();
static void set_next_mode(MODE_ID _id, bool force);
static void set_move_cmd(MSG_ReqMovePos *_req);
static void set_timeangle_cmd(MSG_ReqMoveTimeAngle *_req);

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  execution_events = xQueueCreate(16, sizeof(ExecutionProgress));
  configASSERT(execution_events != nullptr);
  p_MsgQueueReq = xQueueCreate(ADT_MSG_REQ_BUFFER_SIZE, sizeof(MSG_REQ));
  p_FollowLatest = xQueueCreate(1, sizeof(FollowTarget));
  configASSERT(p_MsgQueueReq != nullptr);
  configASSERT(p_FollowLatest != nullptr);

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
    DEBUG_PRINT_PRC_START(ADT_MAIN);

    /* Message処理 */
    process_message();

    /* ArmFollow is latest-only. A timeout latches it off until a new READY. */
    FollowTarget latest;
    if(xQueueReceive(p_FollowLatest, &latest, 0) == pdPASS &&
       follow_armed && follow_permitted && m_nowProcess == &m_posseq) {
      if(!follow_locked) m_posseq.cancel();
      follow_locked = true;
      follow_active = true;
      follow_target = latest;
      follow_received_ms = millis();
    }
    if(follow_active &&
       (uint32_t)(millis() - follow_received_ms) > FOLLOW_TIMEOUT_MS) {
      // Hold the last accepted position. Never extrapolate after link loss.
      follow_active = false;
      follow_armed = false;
      follow_permitted = false;
      m_posseq.cancel();
    }

    /* Follow holds the latest command; stale commands hold the final pose. */
    if(!follow_locked) m_nowProcess->update();
    if(follow_active && m_nowProcess == &m_posseq) {
      for(int i = 0; i < JointAxis::J_NUM; ++i) {
        ADTModeBase::P_JOINT_[i]->set_tgt_ang_deg(follow_target.rad[i] * 57.2957795f);
      }
    }

    // Report actual AD task readiness; receiving Command=12 alone is not readiness.
    manual_state_value = !manual_enabled ? 0 :
      (m_nowProcess == &m_posseq && !m_posseq.isCompleted()) ? 3 :
      (m_nowProcess == &m_posseq || (m_nowProcess == &m_init_posmove && m_nowProcess->isCompleted())) ? 2 : 1;

    /* 面向けYaw補正の更新(トルクON中は全Mode共通で常時適用) */
    update_yaw_correction();

    /* 胴向けZ補正の更新(PositioningSeq中、CADモデル有効時) */
    update_z_correction();

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
      //debug_printf("%d,%d,%d\n",(int)j_P1.get_tgt_deg(),(int)j_P1.get_now_deg(), (int)(j_P1.get_now_cur()*1000.0f));
      // debug_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", millis(), // 
      //                                                   (int)j_Y0.get_tgt_deg(),(int)j_Y0.get_now_deg(), //
      //                                                   (int)j_P1.get_tgt_deg(),(int)j_P1.get_now_deg(), //
      //                                                   (int)j_P2.get_tgt_deg(),(int)j_P2.get_now_deg(), //
      //                                                   (int)j_R0.get_tgt_deg(),(int)j_R0.get_now_deg(), //
      //                                                   (int)j_P3.get_tgt_deg(),(int)j_P3.get_now_deg());
      // DEBUG_PRINT_ADT("[ADT]%d,%d\n", (int)j_P1.get_tgt_deg(), (int)(j_P1.get_now_deg()));
      // debug_printf("[ADT]Tgt:%d,%d,%d,%d,%d\n", (int)j_Y0.get_tgt_deg(), (int)j_P1.get_tgt_deg(), (int)j_P2.get_tgt_deg(), (int)j_R0.get_tgt_deg(), (int)j_P3.get_tgt_deg());
      // DEBUG_PRINT_ADT("[ADT]Pos:%d,%d,%d,%d,%d\n", (int)j_Y0.get_now_deg(), (int)j_P1.get_now_deg(), (int)j_DF_Left.get_now_deg(), (int)j_DF_Right.get_now_deg(), (int)j_P3.get_now_deg());
      // debug_printf("%d,%d,%d,%d\n", (int)j_P2.get_tgt_deg(), (int)j_R0.get_tgt_deg(), (int)j_P2.get_now_deg(), (int)j_R0.get_now_deg());

      counter = 0;
    } else {
      counter++;
    }
    // DEBUG_PRINT_ADT("[ADT]%d,%d\n", micros(), (int)j_Y0.get_now_deg());
    //
    DEBUG_PRINT_PRC_FINISH(ADT_MAIN);
  }
}

/**
 * @brief
 *
 */
static void process_message() {
  taskENTER_CRITICAL();
  const bool stop = emergency_stop_requested;
  const int8_t manual_request = manual_mode_request;
  emergency_stop_requested = false;
  manual_mode_request = -1;
  taskEXIT_CRITICAL();
  if(stop) {
    // STOP bypasses the normal mailbox, even when it is full. Discard pending
    // requests so an older MOVE_START cannot restart a stopped trajectory.
    set_next_mode(MODE_ID::OFF, true);
    while(xQueueReceive(p_MsgQueueReq, &msgReq, 0) == pdPASS) {
      if(msgReq.common.MsgId == MSG_ID::REQ_MOVE_TIMEANGLE) {
        report_execution_progress(msgReq.time_angle.u32_id, 0, 2);
      }
    }
    // A manual request received after STOP changes only the mode, never torque.
    if(manual_request >= 0) manual_enabled = manual_request != 0;
    return;
  }
  if(manual_request >= 0) manual_enabled = manual_request != 0;
  if(xQueueReceive(p_MsgQueueReq, &msgReq, 0) == pdPASS) {
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
      set_timeangle_cmd(&msgReq.time_angle);
      break;
    case MSG_ID::REQ_FOLLOW_ARM:
      m_posseq.cancel();
      xQueueReset(p_FollowLatest);
      follow_armed = true;
      follow_permitted = false;
      follow_active = false;
      follow_received_ms = 0;
      follow_locked = false;
      break;
    case MSG_ID::REQ_FOLLOW_START:
      if(follow_armed) follow_permitted = true;
      break;
    case MSG_ID::REQ_FOLLOW_RELEASE:
      xQueueReset(p_FollowLatest);
      follow_active = false;
      follow_armed = false;
      follow_permitted = false;
      m_posseq.cancel();
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
 * @brief 面向けYaw補正(J0)の更新
 *
 * トルクON中は全Mode共通で常時適用する。補正はJointBaseの出力段オフセットとして持たせるため、
 * Modeが見る座標系(get_tgt_deg/get_now_base_deg)には現れず、Mode側の実装は一切不要。
 * J0_YAWは角度初期化(MODE:INIT)の端当て・原点リセット対象外(AD_mode_initialize.cpp:81,100)なので
 * INITを特別扱いする必要はない。MODE:OFFはトルクOFFのため自然に除外される。
 */
static void update_yaw_correction() {
  if(!j_Y0.get_torque_on()) {
    /* トルクOFF中(サーボ脱力中)は補正を持ち越さない */
    yaw_corr_lpf.reset();
    j_Y0.set_corr_ang_deg(0.0f);
    return;
  }

  /* 受信が途絶えた場合は0へ戻す(古い補正を保持し続けない) */
  float _raw = (follow_locked || get_yaw_corr_age_ms() > U32_YAW_STALE_MS) ? 0.0f : get_yaw_corr_deg();
  _raw       = UTIL::mymath::satf(_raw, FL_YAW_CORR_MAX_DEG, -FL_YAW_CORR_MAX_DEG);

  /* LPF → レート制限 の順で平滑化し、精密サーボへのステップ流入を防ぐ */
  float _lpf  = yaw_corr_lpf.update(_raw);
  float _prev = j_Y0.get_corr_ang_deg();
  float _corr = _prev + UTIL::mymath::satf(_lpf - _prev, FL_YAW_SLEW_DEG, -FL_YAW_SLEW_DEG);

  j_Y0.set_corr_ang_deg(_corr);
}

/**
 * @brief Z補正オフセットと内部状態をクリアする
 */
static void clear_z_correction() {
  z_err_lpf.reset();
  s_fl_z_corr_rad[JointAxis::J0_YAW] = 0.0f; // J0はYaw補正と競合するため常に未使用
  for(int i = JointAxis::J1_PITCH; i < JointAxis::J_NUM; ++i) {
    s_fl_z_corr_rad[i] = 0.0f;
    ADTModeBase::P_JOINT_[i]->set_corr_ang_deg(0.0f);
  }
}

// モード切替時は最大20度の補正を1周期で外さず、通常更新と同じ速度で戻す。
static void release_z_correction() {
  z_err_lpf.reset();
  for(int i = JointAxis::J1_PITCH; i < JointAxis::J_NUM; ++i) {
    const float step = UTIL::mymath::satf(-s_fl_z_corr_rad[i], FL_Z_SLEW_RAD, -FL_Z_SLEW_RAD);
    s_fl_z_corr_rad[i] += step;
    ADTModeBase::P_JOINT_[i]->set_corr_ang_deg(s_fl_z_corr_rad[i] / FL_RAD_PER_DEG);
  }
}

/**
 * @brief 胴向けZ補正の更新
 *
 * update_yaw_correction() と同様にMode更新後、関節通信前に毎周期実行する。
 * PositioningSeqが生成した補正抜き目標をFKへ通し、Z方向のDLS補正を
 * 各関節の出力段オフセットとして適用する。J0はYaw補正と競合するため使わない。
 */
static void update_z_correction() {
  const bool p1_on = j_P1.get_torque_on();
  const bool p2_on = j_P2.get_torque_on();
  const bool roll_on = j_R0.get_torque_on();
  const bool p3_on = j_P3.get_torque_on();
  if(!is_z_correction_available() || !(p1_on || p2_on || roll_on || p3_on)) {
    // 全軸トルクOFFなら指令による急動作は起きない。
    clear_z_correction();
    return;
  }
  if(follow_locked || m_nowProcess != &m_posseq || !(p1_on && p2_on && roll_on && p3_on)) {
    release_z_correction();
    return;
  }

  /* Yawと同じく、入力が古い場合は目標誤差を0へ戻す。 */
  const float raw_err_m = (get_height_err_age_ms() > U32_Z_STALE_MS) ? 0.0f : get_height_err();
  const float filtered_err_m = z_err_lpf.update(raw_err_m);

  float base_rad[JointAxis::J_NUM];
  float corrected_rad[JointAxis::J_NUM];
  for(int i = 0; i < JointAxis::J_NUM; ++i) {
    base_rad[i] = ADTModeBase::P_JOINT_[i]->get_tgt_deg() * FL_RAD_PER_DEG;
    corrected_rad[i] = base_rad[i] + s_fl_z_corr_rad[i];
  }

  Kinematics::Result base_fk = {};
  Kinematics::Result corrected_fk = {};
  const Kinematics::ModelConfig &model = Kinematics::robot_model_config();
  if(!Kinematics::calculate(model, base_rad, &base_fk) ||
     !Kinematics::calculate(model, corrected_rad, &corrected_fk)) {
    release_z_correction();
    return;
  }

  const float z_target_m = base_fk.tool_position_m.z + filtered_err_m;
  const float z_error_m = z_target_m - corrected_fk.tool_position_m.z;
  float jz_norm_sq = 0.0f;
  for(int i = JointAxis::J1_PITCH; i < JointAxis::J_NUM; ++i) {
    jz_norm_sq += corrected_fk.position_jacobian_z[i] * corrected_fk.position_jacobian_z[i];
  }
  const float denom = jz_norm_sq + FL_Z_DLS_LAMBDA_MPRAD * FL_Z_DLS_LAMBDA_MPRAD;

  for(int i = JointAxis::J1_PITCH; i < JointAxis::J_NUM; ++i) {
    const float delta_rad = FL_Z_CLIK_GAIN * corrected_fk.position_jacobian_z[i] * z_error_m / denom;
    const float slewed_rad = UTIL::mymath::satf(delta_rad, FL_Z_SLEW_RAD, -FL_Z_SLEW_RAD);
    s_fl_z_corr_rad[i] = UTIL::mymath::satf(s_fl_z_corr_rad[i] + slewed_rad,
                                             FL_Z_CORR_MAX_RAD, -FL_Z_CORR_MAX_RAD);
    ADTModeBase::P_JOINT_[i]->set_corr_ang_deg(s_fl_z_corr_rad[i] / FL_RAD_PER_DEG);
  }

  DEBUG_PRINT_ADT("[ADT]ZCorr z_mm:%d err_um:%d jz_umrad:%d,%d,%d,%d dq_mrad:%d,%d,%d,%d\\n",
                  (int)(corrected_fk.tool_position_m.z * 1000.0f), (int)(z_error_m * 1000000.0f),
                  (int)(corrected_fk.position_jacobian_z[JointAxis::J1_PITCH] * 1000000.0f),
                  (int)(corrected_fk.position_jacobian_z[JointAxis::J2_PITCH] * 1000000.0f),
                  (int)(corrected_fk.position_jacobian_z[JointAxis::J3_ROLL] * 1000000.0f),
                  (int)(corrected_fk.position_jacobian_z[JointAxis::J4_PITCH] * 1000000.0f),
                  (int)(s_fl_z_corr_rad[JointAxis::J1_PITCH] * 1000.0f),
                  (int)(s_fl_z_corr_rad[JointAxis::J2_PITCH] * 1000.0f),
                  (int)(s_fl_z_corr_rad[JointAxis::J3_ROLL] * 1000.0f),
                  (int)(s_fl_z_corr_rad[JointAxis::J4_PITCH] * 1000.0f));
}

/**
 * @brief Set the next mode object
 *
 * @param _id
 */
static void set_next_mode(MODE_ID _id, bool force) {
  if(_id == MODE_ID::OFF) {
    manual_enabled = false;
    manual_state_value = 0;
    follow_active = false;
    follow_armed = false;
    follow_permitted = false;
    follow_locked = false;
    m_posseq.cancel();
    xQueueReset(p_FollowLatest);
  }
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
  if(manual_enabled && m_nowProcess == &m_posseq && !m_posseq.isCompleted()) {
    report_execution_progress(_req->u32_id, 0, 2);
    return;
  }
  // A completed READY is sufficient in manual mode; never issue autonomous START.
  if(manual_enabled && m_nowProcess == &m_init_posmove && m_nowProcess->isCompleted())
    set_next_mode(MODE_ID::POSITIONING_SEQ, false);
  if(follow_locked) {
    report_execution_progress(_req->u32_id, 0, 2);
    return;
  }
  if(m_nowProcess != &m_posseq) {
    report_execution_progress(_req->u32_id, 0, 2);
    return;
  }

  ADTModePositioningSeq::PosCmdSeq _cmdseq = {};

  _cmdseq.u32_id         = _req->u32_id;
  _cmdseq.u8_cmd_seq_len = _req->u32_len;

  for(int i = 0; i < (int)_req->u32_len; i++) {
    _cmdseq.cmd_seq[i].u32_dt_ms         = _req->points[i].dt_ms;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[0] = _req->points[i].theta_rad[0] * 57.29578f; // rad to deg
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[1] = _req->points[i].theta_rad[1] * 57.29578f;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[2] = _req->points[i].theta_rad[2] * 57.29578f;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[3] = _req->points[i].theta_rad[3] * 57.29578f;
    _cmdseq.cmd_seq[i].fl_tgt_pos_deg[4] = _req->points[i].theta_rad[4] * 57.29578f;
  }

  m_posseq.push_cmdseq(_cmdseq);
}

/**
 * @brief
 *
 * @param _msg
 */
bool set_follow_target_rad(const float *angles) {
  if(!p_FollowLatest) return false;
  FollowTarget target = {};
  for(int i = 0; i < 5; ++i) {
    if(!isfinite(angles[i]) || angles[i] < FOLLOW_MIN_RAD[i] - FOLLOW_LIMIT_EPS ||
       angles[i] > FOLLOW_MAX_RAD[i] + FOLLOW_LIMIT_EPS) return false;
    target.rad[i] = fminf(FOLLOW_MAX_RAD[i], fmaxf(FOLLOW_MIN_RAD[i], angles[i]));
  }
  return xQueueOverwrite(p_FollowLatest, &target) == pdPASS;
}

bool is_follow_locked() { return follow_locked; }

bool send_req_msg(const MSG_REQ *_msg) {
  if(_msg->common.MsgId == MSG_ID::REQ_MANUAL_MODE) {
    taskENTER_CRITICAL();
    manual_mode_request = _msg->common.Recv0 != 0;
    taskEXIT_CRITICAL();
    return true;
  }
  if(_msg->common.MsgId == MSG_ID::REQ_CHANGE_MODE &&
     _msg->change_mode.u32_mode_id == MODE_ID::OFF && _msg->change_mode.u8_forced) {
    taskENTER_CRITICAL();
    emergency_stop_requested = true;
    manual_mode_request = 0;
    taskEXIT_CRITICAL();
    return true;
  }
  const bool sent = xQueueSend(p_MsgQueueReq, _msg, 0) == pdPASS;
  if(!sent) {
    if(_msg->common.MsgId == MSG_ID::REQ_MOVE_TIMEANGLE) {
      report_execution_progress(_msg->time_angle.u32_id, 0, 2);
    }
    Serial.println("[ADT] request queue full; dropped");
  }
  return sent;
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

/**
 * @brief 現在のArm関節角度を返す(即時回答)
 *
 * @param _fl_ang : [out] 配列5の角度[rad]
 */
void get_arm_angle_rad(float *_fl_ang){
  _fl_ang[0] = j_Y0.get_now_deg()*0.0174533f; // deg to rad
  _fl_ang[1] = j_P1.get_now_deg()*0.0174533f; // deg to rad
  _fl_ang[2] = j_P2.get_now_deg()*0.0174533f; // deg to rad
  _fl_ang[3] = j_R0.get_now_deg()*0.0174533f; // deg to rad
  _fl_ang[4] = j_P3.get_now_deg()*0.0174533f; // deg to rad
}

/**
 * @brief 補正入力の書き込み/読み出し(RMTタスク→ADTタスク)
 */
void set_yaw_corr_deg(float _deg) {
  s_fl_yaw_corr_deg = _deg;
  s_u32_yaw_corr_ms = millis();
}
void set_height_err(float _err) {
  // 不正な入力でFK/Jacobianやモータ指令をNaNへ伝播させない。
  if(!isfinite(_err)) return;
  s_fl_height_err     = _err;
  s_u32_height_err_ms = millis();
}
float    get_yaw_corr_deg() { return s_fl_yaw_corr_deg; }
uint32_t get_yaw_corr_age_ms() { return (uint32_t)(millis() - s_u32_yaw_corr_ms); }
float    get_height_err() { return s_fl_height_err; }
uint32_t get_height_err_age_ms() { return (uint32_t)(millis() - s_u32_height_err_ms); }
bool is_z_correction_available() { return Kinematics::is_valid(Kinematics::robot_model_config()); }

}; // namespace ADT
