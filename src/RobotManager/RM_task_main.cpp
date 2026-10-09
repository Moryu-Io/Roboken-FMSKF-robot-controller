// RTOS
#include <FreeRTOS_TEENSY4.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <interfaces/msg/arm_info.h>
#include <interfaces/msg/arm_execution_progress.h>
#include <interfaces/msg/cam_angle_order.h>
#include <interfaces/msg/command.h>
#include <interfaces/msg/mecanum_command.h>
#include <interfaces/msg/mecanum_cont_order.h>
#include <interfaces/msg/time_angle.h>
#include <interfaces/msg/vehicle_info.h>
#include <interfaces/srv/proc_status.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <rosidl_runtime_c/string_functions.h>
#include <stdio.h>

#include "../ArmDrive/AD_task_main.hpp"
#include "../CameraGimbal/CG_task_main.hpp"
#include "../FloorDetect/FD_task_main.hpp"
#include "../Utility/util_led.hpp"
#include "../Utility/util_mymath.hpp"
#include "../VehicleDrive/VD_task_main.hpp"
#include "../Imu/imu_task_main.hpp"

// Local status
/* Ethernet */
enum ConnectionStatus{
  ETHERNET_INIT,
  WAITING_AGENT,
  AVAILABLE_AGENT,
  CONNECTED,
  DISCONNECTED,
  UNKNOWN,
};
ConnectionStatus UROS_AGENT_STATUS = ETHERNET_INIT;
uint32_t U32_UROS_PING_COUNTER_MATCH = 15;  // この回数に一回、Pingを打つ
uint32_t U32_UROS_PING_COUNTER       = 0; // Pingを打つまでのカウンター

/* CommandIF */
enum CmdStatus {
  RELAX       = 0,
  MOVE_READY  = 1,
  MOVE_START  = 2,
  QUIT_PG     = 3,
  INIT        = 4,
  HW_DEBUG    = 5,
  DUUMY6      = 6,
  DUUMY7      = 7,
  DUUMY8      = 8,
  DUUMY9      = 9,
  SWITCH_FLOOR_SENSOR = 10,
  UNKNOWN_CMD = 0xFF,
};
CmdStatus NOW_CMD_STATUS = CmdStatus::RELAX;

uint32_t     U32_MCN_NO_CMD_CNT            = 0;
uint32_t     U32_MCN_NO_CMD_STOP_THRE      = 200;
uint32_t     U32_MCN_WALL_LEAVE_TIME_MS    = 200; // 壁から離れる時の駆動時間
uint32_t     U32_MCN_WALL_LEAVE_SPEED_MMPS = 100; // 壁から離れる時の速度
bool         IS_MCN_CMD_UPDATED            = false;
uint8_t      U8_VDT_MSG_BUF_WRITE          = 0;   // 現在書き込み対象のBuffer面
VDT::MSG_REQ vdt_msg_buf_[2];
bool         IS_IGNORE_FLOOR_DETECTION = false; // 床検知を無視するかどうかのフラグ

typedef union{
  uint32_t val;
  struct{
    uint32_t wall_abort_vdt_x_p:1;  // 壁起因でX+方向車両駆動リクエストが棄却されたかどうか
    uint32_t wall_abort_vdt_x_m:1;  // 壁起因でX-方向車両駆動リクエストが棄却されたかどうか
    uint32_t wall_abort_vdt_y_p:1;  // 壁起因でY+方向車両駆動リクエストが棄却されたかどうか
    uint32_t wall_abort_vdt_y_m:1;  // 壁起因でY-方向車両駆動リクエストが棄却されたかどうか
    uint32_t wall_abort_vdt_r_p:1;  // 壁起因でR+方向車両駆動リクエストが棄却されたかどうか
    uint32_t wall_abort_vdt_r_m:1;  // 壁起因でR-方向車両駆動リクエストが棄却されたかどうか
    uint32_t resv_6_7:2;            // resrv
    uint32_t fllr_abort_vdt_x_p:1;  // 床起因でX+方向車両駆動リクエストが棄却されたかどうか(Dir)
    uint32_t fllr_abort_vdt_x_m:1;  // 床起因でX-方向車両駆動リクエストが棄却されたかどうか(Dir)
    uint32_t fllr_abort_vdt_y_p:1;  // 床起因でY+方向車両駆動リクエストが棄却されたかどうか(Dir)
    uint32_t fllr_abort_vdt_y_m:1;  // 床起因でY-方向車両駆動リクエストが棄却されたかどうか(Dir)
    uint32_t fllr_abort_vdt_r_p:1;  // 床起因でR+方向車両駆動リクエストが棄却されたかどうか(Dir)
    uint32_t fllr_abort_vdt_r_m:1;  // 床起因でR-方向車両駆動リクエストが棄却されたかどうか(Dir)
    uint32_t resv_14_15:2;         // resrv
    uint32_t fllr_abort_vdt_cont_trans_dir:1;  // 床起因で連続駆動進行方向車両駆動リクエストが棄却されたかどうか
    uint32_t fllr_abort_vdt_cont_rot_dir:1;    // 床起因で連続駆動回転方向車両駆動リクエストが棄却されたかどうか
    uint32_t resv_18_31:14;         // resrv
  }bit;
}VDT_REQ_ABORT;
VDT_REQ_ABORT vdt_abort = {};

// publisher
rcl_publisher_t              pb_vchlInfo;
rcl_publisher_t              pb_ArmInfo;
rcl_publisher_t              pb_ArmExecutionProgress;
interfaces__msg__VehicleInfo msg_pb_vhclInfo;
interfaces__msg__ArmInfo     msg_pb_armInfo;
interfaces__msg__ArmExecutionProgress msg_pb_executionProgress = {};

uint8_t U8_PUB_PHASE = 0;

// Buffer for ArmInfo
static constexpr uint8_t U8_ARMANGLE_BUF_LEN = 5;
float fl_ArmAngThetaBuffer[U8_ARMANGLE_BUF_LEN] = {};

// subscriber
rcl_subscription_t              sb_mcnmCmd;
rcl_subscription_t              sb_mcnmContOdr;
rcl_subscription_t              sb_mcnmCmdVel;
rcl_subscription_t              sb_tmAngle;
bool is_arm_follow_armed = false;
bool is_arm_follow_started = false;
bool is_arm_follow_locked = false;
rcl_subscription_t              sb_cmd;
rcl_subscription_t              sb_camAngOdr;
rcl_subscription_t              sb_armCorr;
interfaces__msg__MecanumCommand   msg_sb_mcnmCmd;
interfaces__msg__MecanumContOrder msg_sb_mcnmContOdr;
geometry_msgs__msg__Twist         msg_sb_mcnmCmdVel;
interfaces__msg__TimeAngle        msg_sb_tmAngle;
interfaces__msg__Command          msg_sb_cmd;
interfaces__msg__CamAngleOrder    msg_sb_camAngOdr;
geometry_msgs__msg__Vector3       msg_sb_armCorr;

// Buffer for TimeAngle
static constexpr uint8_t U8_TIMEANGLE_BUF_LEN                     = 32;
interfaces__msg__Joint   TimeAngleBuffer[5][U8_TIMEANGLE_BUF_LEN] = {};

// service
rcl_service_t                        srv_proc;
interfaces__srv__ProcStatus_Request  srv_req_procSts;
interfaces__srv__ProcStatus_Response srv_res_procSts;

rclc_executor_t executor;
rclc_executor_t executor_srv;

rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     timer;

bool is_support_initialized       = false;
bool is_node_initialized          = false;
bool is_pb_vchl_initialized       = false;
bool is_pb_arm_initialized        = false;
bool is_pb_progress_initialized = false;
bool is_progress_message_initialized = false;
bool is_sb_mcnm_initialized       = false;
bool is_sb_tm_angle_initialized   = false;
bool is_sb_cmd_initialized        = false;
bool is_sb_cam_initialized        = false;
bool is_sb_arm_corr_initialized   = false;
bool is_service_initialized       = false;
bool is_executor_initialized      = false;
bool is_executor_srv_initialized  = false;

#define toggleRMtaskLED UTIL::toggle_LED1

static void report_rcl_failure(const char *operation, rcl_ret_t rc) {
  const rcl_error_string_t error = rcl_get_error_string();
  Serial.printf("[RMT] %s failed: %d (%s)\n", operation, (int)rc, error.str);
  rcl_reset_error();
}

static void report_rmw_failure(const char *operation, rmw_ret_t rc) {
  Serial.printf("[RMT] %s failed: %d\n", operation, (int)rc);
  rcl_reset_error();
}

static void reset_microros_handles() {
  support       = {};
  node          = rcl_get_zero_initialized_node();
  pb_vchlInfo   = rcl_get_zero_initialized_publisher();
  pb_ArmInfo    = rcl_get_zero_initialized_publisher();
  pb_ArmExecutionProgress = rcl_get_zero_initialized_publisher();
  sb_mcnmContOdr = rcl_get_zero_initialized_subscription();
  sb_mcnmCmdVel = rcl_get_zero_initialized_subscription();
  sb_tmAngle    = rcl_get_zero_initialized_subscription();
  sb_cmd        = rcl_get_zero_initialized_subscription();
  sb_camAngOdr  = rcl_get_zero_initialized_subscription();
  sb_armCorr    = rcl_get_zero_initialized_subscription();
  srv_proc      = rcl_get_zero_initialized_service();
  executor      = rclc_executor_get_zero_initialized_executor();
  executor_srv  = rclc_executor_get_zero_initialized_executor();
  is_support_initialized = is_node_initialized = false;
  is_pb_vchl_initialized = is_pb_arm_initialized = false;
  is_pb_progress_initialized = is_progress_message_initialized = false;
  is_sb_mcnm_initialized = is_sb_tm_angle_initialized = false;
  is_sb_cmd_initialized = is_sb_cam_initialized = is_sb_arm_corr_initialized = false;
  is_service_initialized = is_executor_initialized = is_executor_srv_initialized = false;
}

#define RMT_CREATE(operation, expression)                     \
  do {                                                         \
    const rcl_ret_t rc = (expression);                         \
    if(rc != RCL_RET_OK) {                                     \
      report_rcl_failure(operation, rc);                       \
      destroy_microros_entities();                             \
      return rc;                                               \
    }                                                          \
  } while(0)

#define RMT_CLEANUP(operation, expression)                    \
  do {                                                         \
    const rcl_ret_t rc = (expression);                         \
    if(rc != RCL_RET_OK) report_rcl_failure(operation, rc);    \
  } while(0)

static rcl_ret_t timer_publish(rcl_publisher_t *publisher, const void *message) {
  const rcl_ret_t rc = rcl_publish(publisher, message, NULL);
  if(rc != RCL_RET_OK) {
    report_rcl_failure("rcl_publish", rc);
  }
  return rc;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if(timer != NULL) {
    timer_publish(&pb_vchlInfo, &msg_pb_vhclInfo);
    msg_pb_vhclInfo.pos.x++;
    msg_pb_vhclInfo.pos.y--;
    msg_pb_vhclInfo.pos.theta += 0.1f;
  }
}

void sb_cmd_callback(const void *msgin) {
  const interfaces__msg__Command *msg = (const interfaces__msg__Command *)msgin;

  /* Command受信時は現状常に車体STOPさせる */
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_dir.u32_time_ms = 1;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_dir.u32_speed   = 0;
  U8_VDT_MSG_BUF_WRITE = U8_VDT_MSG_BUF_WRITE ^ 1; 
  IS_MCN_CMD_UPDATED   = true;

  NOW_CMD_STATUS = (CmdStatus)msg->command;
  ADT::MSG_REQ follow_request = {};
  if(msg->command == CmdStatus::MOVE_READY) {
    is_arm_follow_armed = true;
    is_arm_follow_started = false;
    is_arm_follow_locked = false;
    follow_request.common.MsgId = ADT::MSG_ID::REQ_FOLLOW_ARM;
    ADT::send_req_msg(&follow_request);
  } else if(msg->command == CmdStatus::MOVE_START && is_arm_follow_armed) {
    is_arm_follow_started = true;
    follow_request.common.MsgId = ADT::MSG_ID::REQ_FOLLOW_START;
    ADT::send_req_msg(&follow_request);
  } else if(msg->command != CmdStatus::MOVE_START) {
    is_arm_follow_armed = false;
    is_arm_follow_started = false;
    is_arm_follow_locked = false;
    follow_request.common.MsgId = ADT::MSG_ID::REQ_FOLLOW_RELEASE;
    ADT::send_req_msg(&follow_request);
  }
  switch(msg->command) {
  case CmdStatus::QUIT_PG:
  case CmdStatus::RELAX: {
    /* ArmトルクOFF */
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::OFF;
    adt_msg.change_mode.u8_forced   = 1;
    ADT::send_req_msg(&adt_msg);
  } break;
  case CmdStatus::MOVE_READY: {
    /* Arm初期位置移動 */
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::INIT_POS_MOVE;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);

    /* カメラジンバルデフォルト位置移動 */
    CGT::MSG_REQ cgt_msg;
    cgt_msg.common.MsgId = CGT::MSG_ID::REQ_DEFAULT_PITCH;
    CGT::send_req_msg(&cgt_msg);
  } break;
  case CmdStatus::MOVE_START: {
    /* Arm初期位置移動 */
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::POSITIONING_SEQ;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);
  } break;
  case CmdStatus::INIT: {
    /* Arm角度初期化(キャリブレーション) */
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::INIT;
    adt_msg.change_mode.u8_forced   = 0;
    ADT::send_req_msg(&adt_msg);

    /* カメラジンバル初期化 */
    CGT::MSG_REQ cgt_msg;
    cgt_msg.common.MsgId = CGT::MSG_ID::REQ_INIT;
    CGT::send_req_msg(&cgt_msg);
  } break;
  case CmdStatus::SWITCH_FLOOR_SENSOR:
    IS_IGNORE_FLOOR_DETECTION = !IS_IGNORE_FLOOR_DETECTION;
    break;
  default:
    NOW_CMD_STATUS = CmdStatus::UNKNOWN_CMD;
    break;
  }

  DEBUG_PRINT_RMT("[RMT]Command:%d\n", msg->command);
}

void sb_mecanumCmd_callback(const void *msgin) {
  const interfaces__msg__MecanumCommand *msg = (const interfaces__msg__MecanumCommand *)msgin;

  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_dir.u32_cmd     = msg->cmd;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_dir.u32_time_ms = msg->time;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_dir.u32_speed   = msg->speed;

  U8_VDT_MSG_BUF_WRITE = U8_VDT_MSG_BUF_WRITE ^ 1;
  IS_MCN_CMD_UPDATED   = true;

  DEBUG_PRINT_STR_RMT("[RMT]McnmCmd\n");
}

void sb_mecanumContOdr_callback(const void *msgin) {
  const interfaces__msg__MecanumContOrder *msg = (const interfaces__msg__MecanumContOrder *)msgin;

  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].common.MsgId = VDT::MSG_ID::REQ_MOVE_CONT_DIR;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.fl_vel_x_mmps = msg->speed.linear.x;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.fl_vel_y_mmps = msg->speed.linear.y;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.fl_vel_th_radps = msg->speed.angular.z;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.u32_time_ms = msg->time_ms;

  U8_VDT_MSG_BUF_WRITE = U8_VDT_MSG_BUF_WRITE ^ 1;
  IS_MCN_CMD_UPDATED   = true;

  DEBUG_PRINT_STR_RMT("[RMT]McnmContOrder\n");
}

void sb_mecanumCmdVel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].common.MsgId = VDT::MSG_ID::REQ_MOVE_CONT_DIR;
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.fl_vel_x_mmps = static_cast<float>(msg->linear.x*1000.0);
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.fl_vel_y_mmps = static_cast<float>(msg->linear.y*1000.0);
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.fl_vel_th_radps = static_cast<float>(msg->angular.z);
  vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE].move_cont_dir.u32_time_ms = 500;  // temporary

  U8_VDT_MSG_BUF_WRITE = U8_VDT_MSG_BUF_WRITE ^ 1;
  IS_MCN_CMD_UPDATED   = true;

  DEBUG_PRINT_STR_RMT("[RMT]CmdVel\n");
}


static constexpr uint32_t MEN_FOLLOW_ID = 0xFFFFFFFEu;
// Compile-time opt-in: the old firmware would interpret this ID as a trajectory.
#ifndef ENABLE_MEN_CHASER_FOLLOW
#define ENABLE_MEN_CHASER_FOLLOW 0
#endif

void sb_armFollow_callback(const interfaces__msg__TimeAngle *msg) {
#if ENABLE_MEN_CHASER_FOLLOW
  bool all_empty = true;
  bool all_single = true;
  float angles[5] = {};
  for(int i = 0; i < 5; ++i) {
    const auto &points = msg->arm[i].point;
    all_empty &= points.size == 0;
    all_single &= points.size == 1 && points.data && points.data[0].dt == 0;
    if(points.size == 1 && points.data) angles[i] = points.data[0].theta;
  }
  if(all_empty) {
    ADT::MSG_REQ request = {};
    request.common.MsgId = ADT::MSG_ID::REQ_FOLLOW_RELEASE;
    ADT::send_req_msg(&request);
    return;
  }
  if(!all_single || !is_arm_follow_started) return;
  if(ADT::set_follow_target_rad(angles)) is_arm_follow_locked = true;
#else
  (void)msg;
#endif
}

void sb_timeAngle_callback(const void *msgin) {
  const interfaces__msg__TimeAngle *msg = (const interfaces__msg__TimeAngle *)msgin;
  if(msg->id == MEN_FOLLOW_ID) {
    sb_armFollow_callback(msg);
    return;
  }
  if(is_arm_follow_locked || ADT::is_follow_locked()) {
    ADT::report_execution_progress(msg->id, 0, 2);
    return;
  }

  DEBUG_PRINT_STR_RMT("[RMT]TimeAngRecv\n");

  // Firmware stores at most 32 synchronous points for all five joints.
  const size_t length = msg->arm[0].point.size;
  if(length == 0 || length > U8_TIMEANGLE_BUF_LEN) {
    ADT::report_execution_progress(msg->id, 0, 2);
    return;
  }
  for(size_t axis = 0; axis < 5; ++axis) {
    if(msg->arm[axis].point.size != length) {
      ADT::report_execution_progress(msg->id, 0, 2);
      return;
    }
    for(size_t point = 0; point < length; ++point) {
      if(msg->arm[axis].point.data[point].dt != msg->arm[0].point.data[point].dt ||
         (point > 0 && msg->arm[axis].point.data[point].dt <
                       msg->arm[axis].point.data[point - 1].dt)) {
        ADT::report_execution_progress(msg->id, 0, 2);
        return;
      }
    }
  }

  /* 二重受け取り防止用処理 */
  const uint32_t CU32_TANG_ID_NO_DATA = 99;
  uint32_t       _u32_sts             = ADT::get_status_timeangle_proc(msg->id);
  if(_u32_sts == CU32_TANG_ID_NO_DATA) {
    // 受け取ったデータがキューのどこにも存在しない場合のみ受け付ける
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId        = ADT::MSG_ID::REQ_MOVE_TIMEANGLE;
    adt_msg.time_angle.u32_id   = msg->id;
    adt_msg.time_angle.u32_len  = msg->arm[0].point.size;
    for(size_t point = 0; point < length; ++point) {
      adt_msg.time_angle.points[point].dt_ms = msg->arm[0].point.data[point].dt;
      for(size_t axis = 0; axis < 5; ++axis) {
        adt_msg.time_angle.points[point].theta_rad[axis] = msg->arm[axis].point.data[point].theta;
      }
    }

    ADT::send_req_msg(&adt_msg);
  } else {
    // 処理中or処理後の場合は、二重受け取りなので破棄
    DEBUG_PRINT_STR_RMT("[RMT]TimeAng overlap\n");
  }

  DEBUG_PRINT_STR_RMT("[RMT]TimeAngCplt\n");
}

void sb_camAngOdr_callback(const void *msgin) {
  const interfaces__msg__CamAngleOrder *msg = (const interfaces__msg__CamAngleOrder *)msgin;

  CGT::MSG_REQ cgt_msg = {};
  cgt_msg.common.MsgId         = CGT::MSG_ID::REQ_MOVE_PY;
  cgt_msg.move_py.fl_pitch_deg = msg->pitch_deg;
  cgt_msg.move_py.fl_yaw_deg   = msg->yaw_deg;
  CGT::send_req_msg(&cgt_msg);

  DEBUG_PRINT_STR_RMT("[RMT]CamAngleOrder\n");
}

// アーム補正を1トピックに集約: Vector3.x=面向けYaw補正[deg](絶対), y=胴向け高さ誤差
// [上位PCとの契約] 高さ誤差のstale監視(積分暴走防止)は本トピックの受信間隔で行う。
//   よって毎メッセージに「その時点で有効な高さ誤差」を必ず載せること(補正不要時は y=0)。
//   yawのみ高速publishしつつ y に古い値を残すと、積分が古い誤差を進め続けてしまう。
//   (より堅牢にするなら z を高さ有効フラグに使い、set_height_errの呼び出しを条件化する)
void sb_armCorr_callback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
  ADT::set_yaw_corr_deg((float)msg->x);
  ADT::set_height_err((float)msg->y);
  DEBUG_PRINT_STR_RMT("[RMT]ArmCorr\n");
}

void srv_procSts_callback(const void *reqin, void *resout) {
  const interfaces__srv__ProcStatus_Request *req = (const interfaces__srv__ProcStatus_Request *)reqin;
  interfaces__srv__ProcStatus_Response      *res = (interfaces__srv__ProcStatus_Response *)resout;

  // ArmDriveTaskに実行状況を問い合わせ
  // uint32_t _u32_sts = ADT::get_status_movepos_proc(req->id);
  uint32_t _u32_sts = ADT::get_status_timeangle_proc(req->id);
  res->status       = _u32_sts;

  DEBUG_PRINT_RMT("[RMT]ProcSts%d,%d\n", req->id, res->status);
}

namespace RMT {
static void destroy_microros_entities();

#ifdef USE_HOME_NETWORK
IPAddress device_ip(192, 168, 10, 177);
// IPAddress agent_ip(192, 168, 10, 128);  // Jetson
// IPAddress agent_ip(192, 168, 10, 117); // laptop
IPAddress agent_ip(192, 168, 10, 108); // desktop
#else
IPAddress device_ip(172, 17, 0, 2);
IPAddress agent_ip(172, 17, 0, 1);
#endif
uint16_t agent_port  = 9999;
byte     mac_addr[8] = {};

static void get_dev_mac_addr(byte *_mac) {
  for(uint8_t by = 0; by < 2; by++) _mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
  for(uint8_t by = 0; by < 4; by++) _mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
  // Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
}

static rcl_ret_t create_microros_entities() {
  reset_microros_handles();
  allocator = rcl_get_default_allocator();

  // create init_options
  RMT_CREATE("rclc_support_init", rclc_support_init(&support, 0, NULL, &allocator));
  is_support_initialized = true;

  // create node
  RMT_CREATE("rclc_node_init_default", rclc_node_init_default(&node, "quinque_hw", "", &support));
  is_node_initialized = true;

  RMT_CREATE("ArmExecutionProgress message init",
      interfaces__msg__ArmExecutionProgress__init(&msg_pb_executionProgress)
          ? RCL_RET_OK : RCL_RET_ERROR);
  is_progress_message_initialized = true;

  // create publisher
  RMT_CREATE("VehicleInfo publisher", rclc_publisher_init_best_effort(
      &pb_vchlInfo,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, VehicleInfo),
      "VehicleInfo"));
  is_pb_vchl_initialized = true;

  RMT_CREATE("ArmInfo publisher", rclc_publisher_init_best_effort(
      &pb_ArmInfo,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, ArmInfo),
      "ArmInfo"));
  is_pb_arm_initialized = true;

  RMT_CREATE("ArmExecutionProgress publisher", rclc_publisher_init_default(
      &pb_ArmExecutionProgress, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, ArmExecutionProgress),
      "ArmExecutionProgress"));
  is_pb_progress_initialized = true;

  // create subscriber
  /* MecanumCommand購読はアーム補正TOPIC用に枠を譲渡(RMW購読上限=5・後継はcmd_vel)。
     内部のREQ_MOVE_DIR自律制御(床センサ回避/Debug)には影響しない。
     復活させる場合は下記executor add / finiのコメントアウトも戻すこと。 */
  // RCCHECK(rclc_subscription_init_default(
  //     &sb_mcnmCmd,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, MecanumCommand),
  //     "MecanumCommand"));

#ifdef USE_MECANUM_CONT_CMD
  /* なぜかめちゃくちゃメモリ食う。しかもpaddingが31kbyteもできる。要検証 */
  //RCCHECK(rclc_subscription_init_default(
  //    &sb_mcnmContOdr,
  //    &node,
  //    ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, MecanumContOrder),
  //    "MecanumContOrder"));
#else
  RMT_CREATE("cmd_vel subscription", rclc_subscription_init_default(
      &sb_mcnmCmdVel,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));
#endif
  is_sb_mcnm_initialized = true;

  RMT_CREATE("TimeAngle subscription", rclc_subscription_init_default(
      &sb_tmAngle,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, TimeAngle),
      "TimeAngle"));
  is_sb_tm_angle_initialized = true;


  RMT_CREATE("Command subscription", rclc_subscription_init_default(
      &sb_cmd,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Command),
      "Command"));
  is_sb_cmd_initialized = true;

  RMT_CREATE("CamAngOrder subscription", rclc_subscription_init_default(
      &sb_camAngOdr,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, CamAngleOrder),
      "CamAngOrder"));
  is_sb_cam_initialized = true;

  /* アーム補正(面向けYaw + 胴向け高さ)を1トピックに集約: Vector3.x=yaw[deg], y=height */
  RMT_CREATE("ArmCorrection subscription", rclc_subscription_init_default(
      &sb_armCorr,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "ArmCorrection"));
  is_sb_arm_corr_initialized = true;

  // create service
  RMT_CREATE("ProcStatus service", rclc_service_init_default(
      &srv_proc,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(interfaces, srv, ProcStatus),
      "ProcStatus"));
  is_service_initialized = true;

  // create timer.
  // rcl_timer_t        timer         = rcl_get_zero_initialized_timer();
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //    &timer,
  //    &support,
  //    RCL_MS_TO_NS(timer_timeout),
  //    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RMT_CREATE("subscription executor", rclc_executor_init(&executor, &support.context, 5, &allocator));
  is_executor_initialized = true;
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  // MecanumCommandはアーム補正用に枠を譲渡(上記init参照)
  // RCCHECK(rclc_executor_add_subscription(&executor, &sb_mcnmCmd, &msg_sb_mcnmCmd, &sb_mecanumCmd_callback, ON_NEW_DATA));
#ifdef USE_MECANUM_CONT_CMD
  RMT_CREATE("MecanumContOrder executor", rclc_executor_add_subscription(&executor, &sb_mcnmContOdr, &msg_sb_mcnmContOdr, &sb_mecanumContOdr_callback, ON_NEW_DATA));
#else
  RMT_CREATE("cmd_vel executor", rclc_executor_add_subscription(&executor, &sb_mcnmCmdVel, &msg_sb_mcnmCmdVel, &sb_mecanumCmdVel_callback, ON_NEW_DATA));
#endif
  RMT_CREATE("TimeAngle executor", rclc_executor_add_subscription(&executor, &sb_tmAngle, &msg_sb_tmAngle, &sb_timeAngle_callback, ON_NEW_DATA));
  RMT_CREATE("Command executor", rclc_executor_add_subscription(&executor, &sb_cmd, &msg_sb_cmd, &sb_cmd_callback, ON_NEW_DATA));
  RMT_CREATE("CamAngOrder executor", rclc_executor_add_subscription(&executor, &sb_camAngOdr, &msg_sb_camAngOdr, &sb_camAngOdr_callback, ON_NEW_DATA));
  RMT_CREATE("ArmCorrection executor", rclc_executor_add_subscription(&executor, &sb_armCorr, &msg_sb_armCorr, &sb_armCorr_callback, ON_NEW_DATA));

  executor_srv = rclc_executor_get_zero_initialized_executor();
  RMT_CREATE("service executor", rclc_executor_init(&executor_srv, &support.context, 1, &allocator));
  is_executor_srv_initialized = true;
  RMT_CREATE("ProcStatus executor", rclc_executor_add_service(&executor_srv, &srv_proc, &srv_req_procSts, &srv_res_procSts, srv_procSts_callback));

  /* 時間同期 */
  const rmw_ret_t sync_rc = rmw_uros_sync_session(1000);
  if(sync_rc != RMW_RET_OK) {
    report_rmw_failure("rmw_uros_sync_session", sync_rc);
    destroy_microros_entities();
    return RCL_RET_ERROR;
  }

  /* 可変長MessageのBufferを設定 */
  for(int i = 0; i < 5; i++) {
    msg_sb_tmAngle.arm[i].point.data     = TimeAngleBuffer[i];
    msg_sb_tmAngle.arm[i].point.capacity = U8_TIMEANGLE_BUF_LEN;
  }

  msg_pb_armInfo.servo.theta.data     = &fl_ArmAngThetaBuffer[0];
  msg_pb_armInfo.servo.theta.capacity = U8_ARMANGLE_BUF_LEN;

  DEBUG_PRINT_STR_RMT("[RMT] Micro-ROS initialization complete\n");
  return RCL_RET_OK;
}

static void destroy_microros_entities(){
  is_arm_follow_armed = false;
  is_arm_follow_started = false;
  is_arm_follow_locked = false;
  ADT::MSG_REQ release = {};
  release.common.MsgId = ADT::MSG_ID::REQ_FOLLOW_RELEASE;
  ADT::send_req_msg(&release);
  if(is_support_initialized) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }

  if(is_executor_srv_initialized) rclc_executor_fini(&executor_srv);
  if(is_executor_initialized) rclc_executor_fini(&executor);
  if(is_service_initialized) RMT_CLEANUP("rcl_service_fini", rcl_service_fini(&srv_proc, &node));
  if(is_sb_arm_corr_initialized) RMT_CLEANUP("ArmCorrection fini", rcl_subscription_fini(&sb_armCorr, &node));
  if(is_sb_cam_initialized) RMT_CLEANUP("CamAngOrder fini", rcl_subscription_fini(&sb_camAngOdr, &node));
  if(is_sb_cmd_initialized) RMT_CLEANUP("Command fini", rcl_subscription_fini(&sb_cmd, &node));
  if(is_sb_tm_angle_initialized) RMT_CLEANUP("TimeAngle fini", rcl_subscription_fini(&sb_tmAngle, &node));
  if(is_sb_mcnm_initialized) {
#ifdef USE_MECANUM_CONT_CMD
    RMT_CLEANUP("MecanumContOrder fini", rcl_subscription_fini(&sb_mcnmContOdr, &node));
#else
    RMT_CLEANUP("cmd_vel fini", rcl_subscription_fini(&sb_mcnmCmdVel, &node));
#endif
  }
  if(is_pb_progress_initialized) RMT_CLEANUP("ArmExecutionProgress publisher fini", rcl_publisher_fini(&pb_ArmExecutionProgress, &node));
  if(is_progress_message_initialized) interfaces__msg__ArmExecutionProgress__fini(&msg_pb_executionProgress);
  if(is_pb_arm_initialized) RMT_CLEANUP("ArmInfo publisher fini", rcl_publisher_fini(&pb_ArmInfo, &node));
  if(is_pb_vchl_initialized) RMT_CLEANUP("VehicleInfo publisher fini", rcl_publisher_fini(&pb_vchlInfo, &node));
  if(is_node_initialized) RMT_CLEANUP("rcl_node_fini", rcl_node_fini(&node));
  if(is_support_initialized) rclc_support_fini(&support);
  reset_microros_handles();
}

static rcl_ret_t publish_execution_progress(const ADT::ExecutionProgress &progress) {
  const int64_t stamp_ns = rmw_uros_epoch_nanos();
  msg_pb_executionProgress.header.stamp.sec = stamp_ns / 1000000000LL;
  msg_pb_executionProgress.header.stamp.nanosec = stamp_ns % 1000000000LL;
  msg_pb_executionProgress.id = progress.id;
  msg_pb_executionProgress.elapsed_ms = progress.elapsed_ms;
  msg_pb_executionProgress.status = progress.status;
  return rcl_publish(&pb_ArmExecutionProgress, &msg_pb_executionProgress, NULL);
}

static rcl_ret_t routine_ros(){
    /* Micro-ROS接続完了時のRoutine */

    /* Spin 処理 */
    rcl_ret_t rc = rclc_executor_spin_some(&executor, RCUTILS_US_TO_NS(2000));
    if(rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) return rc;
    rc = rclc_executor_spin_some(&executor_srv, RCUTILS_US_TO_NS(1000));
    if(rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) return rc;

    ADT::ExecutionProgress progress;
    while(ADT::take_execution_event(progress)) {
      rc = publish_execution_progress(progress);
      if(rc != RCL_RET_OK) return rc;
    }
    if(ADT::get_execution_progress(progress)) {
      rc = publish_execution_progress(progress);
      if(rc != RCL_RET_OK) return rc;
    }

    /********** 車体Manage処理 **********/
    bool         _exist_tx_msg = false; // 今回のサイクルで送信するMSGがあるかどうか
    VDT::MSG_REQ vdt_msg;               //送信MSG
    /* まず壁情報の取得 */
    FDT::Info_FloorDetect _st_flrDtct;
    FDT::get_now_FDinfo(_st_flrDtct);

    /* 移動要求があるかを確認する */
    if(IS_MCN_CMD_UPDATED) {
      IS_MCN_CMD_UPDATED = false;
      _exist_tx_msg      = true;

      vdt_abort.val = 0; // Abortフラグ解除

      /* 送信Msgのコピー */
      vdt_msg = vdt_msg_buf_[U8_VDT_MSG_BUF_WRITE^1];  // 読み出し面をコピー
    } else {
      vdt_msg.common.MsgId = VDT::MSG_ID::REQ_MOVE_DIR;
      vdt_msg.move_dir.u32_cmd     = 0;
      vdt_msg.move_dir.u32_time_ms = 0;
      vdt_msg.move_dir.u32_speed   = 0;
    }

    // Temporary: 床センサが同時に「5つ以上床なし」or「5つ以上敵」の場合は無視
    uint8_t u8_floor_buf[8] = {
      _st_flrDtct.u8_rForward,
      _st_flrDtct.u8_lForward,
      _st_flrDtct.u8_rBack,
      _st_flrDtct.u8_lBack,
      _st_flrDtct.u8_right,
      _st_flrDtct.u8_left,
      _st_flrDtct.u8_forward,
      _st_flrDtct.u8_back
    };
    uint8_t u8_no_floor_cnt = 0;
    uint8_t u8_wall_cnt     = 0;
    for(uint8_t i = 0; i < 8; i++) {
      if(u8_floor_buf[i] == NO_DETECTED) u8_no_floor_cnt++;
      else if(u8_floor_buf[i] == WALL_DETECTED) u8_wall_cnt++;
    }

    if((u8_no_floor_cnt >= 5) || (u8_wall_cnt >= 5) || IS_IGNORE_FLOOR_DETECTION) {
      // 床なし多発時は床なし判定を無視
      _st_flrDtct.u8_rForward = FLOOR_DETECTED;
      _st_flrDtct.u8_lForward = FLOOR_DETECTED;
      _st_flrDtct.u8_rBack    = FLOOR_DETECTED;
      _st_flrDtct.u8_lBack    = FLOOR_DETECTED;
      _st_flrDtct.u8_right    = FLOOR_DETECTED;
      _st_flrDtct.u8_left     = FLOOR_DETECTED;
      _st_flrDtct.u8_forward  = FLOOR_DETECTED;
      _st_flrDtct.u8_back     = FLOOR_DETECTED;
    }


    /* 戦闘モードでは相手との距離を離す処理を行う */
#if 1
    if(NOW_CMD_STATUS == CmdStatus::MOVE_START) {
      if(_st_flrDtct.u8_forward == WALL_DETECTED) {
        vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
        vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_BACK;
        vdt_msg.move_dir.u32_time_ms = U32_MCN_WALL_LEAVE_TIME_MS;
        vdt_msg.move_dir.u32_speed   = U32_MCN_WALL_LEAVE_SPEED_MMPS;
        _exist_tx_msg                = true;
        vdt_abort.bit.wall_abort_vdt_x_p = 1;
      } else if(_st_flrDtct.u8_back == WALL_DETECTED) {
        vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
        vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_FORWARD;
        vdt_msg.move_dir.u32_time_ms = U32_MCN_WALL_LEAVE_TIME_MS;
        vdt_msg.move_dir.u32_speed   = U32_MCN_WALL_LEAVE_SPEED_MMPS;
        _exist_tx_msg                = true;
        vdt_abort.bit.wall_abort_vdt_x_m = 1;
      } else if(_st_flrDtct.u8_left == WALL_DETECTED) {
        vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
        vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_RIGHT;
        vdt_msg.move_dir.u32_time_ms = U32_MCN_WALL_LEAVE_TIME_MS;
        vdt_msg.move_dir.u32_speed   = U32_MCN_WALL_LEAVE_SPEED_MMPS;
        _exist_tx_msg                = true;
        vdt_abort.bit.wall_abort_vdt_y_p = 1;
      } else if(_st_flrDtct.u8_right == WALL_DETECTED) {
        vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
        vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::GO_LEFT;
        vdt_msg.move_dir.u32_time_ms = U32_MCN_WALL_LEAVE_TIME_MS;
        vdt_msg.move_dir.u32_speed   = U32_MCN_WALL_LEAVE_SPEED_MMPS;
        _exist_tx_msg                = true;
        vdt_abort.bit.wall_abort_vdt_y_m = 1;
      }
    }

    /* 最後に床の有無からSTOP処理 */
    /* 床検知状態で無い場合はSTOPする */
    if(vdt_msg.common.MsgId == VDT::MSG_ID::REQ_MOVE_DIR){
      switch(vdt_msg.move_dir.u32_cmd) {
      case VDT::REQ_MOVE_DIR_CMD::GO_FORWARD:
        if((_st_flrDtct.u8_forward != FLOOR_DETECTED) ){
          //|| (_st_flrDtct.u8_rForward != FLOOR_DETECTED)
          //|| (_st_flrDtct.u8_lForward != FLOOR_DETECTED)) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_x_p = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_BACK:
        if(_st_flrDtct.u8_back != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_x_m = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_RIGHT:
        if(_st_flrDtct.u8_right != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_y_m = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_LEFT:
        if(_st_flrDtct.u8_left != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_y_p = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_RIGHT_FORWARD:
        if(_st_flrDtct.u8_rForward != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_x_p = 1;
          vdt_abort.bit.fllr_abort_vdt_y_m = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_LEFT_FORWARD:
        if(_st_flrDtct.u8_lForward != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_x_p = 1;
          vdt_abort.bit.fllr_abort_vdt_y_p = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_RIGHT_BACK:
        if(_st_flrDtct.u8_rBack != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_x_m = 1;
          vdt_abort.bit.fllr_abort_vdt_y_m = 1;
        }
        break;
      case VDT::REQ_MOVE_DIR_CMD::GO_LEFT_BACK:
        if(_st_flrDtct.u8_lBack != FLOOR_DETECTED) {
          vdt_msg.common.MsgId       = VDT::MSG_ID::REQ_MOVE_DIR;
          vdt_msg.move_dir.u32_cmd   = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
          vdt_msg.move_dir.u32_time_ms = 1;
          vdt_msg.move_dir.u32_speed = 0;
          _exist_tx_msg              = true;
          vdt_abort.bit.fllr_abort_vdt_x_m = 1;
          vdt_abort.bit.fllr_abort_vdt_y_p = 1;
        }
        break;
      default:
        break;
      }
    }else if(vdt_msg.common.MsgId == VDT::MSG_ID::REQ_MOVE_CONT_DIR){
      if((UTIL::mymath::absf(vdt_msg.move_cont_dir.fl_vel_x_mmps) < 0.01f)
         && (UTIL::mymath::absf(vdt_msg.move_cont_dir.fl_vel_y_mmps) < 0.01f)){

      }else{
        // 進む方向を算出
        float _vph  = UTIL::mymath::atan2f(vdt_msg.move_cont_dir.fl_vel_y_mmps,
                                          vdt_msg.move_cont_dir.fl_vel_x_mmps);
        
        // 4方向優先
        if(_st_flrDtct.u8_forward != FLOOR_DETECTED){
          if(-3.1415f*0.33f < _vph  && _vph <= +3.1415f*0.33f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_back != FLOOR_DETECTED){
          if(+3.1415f*0.66f < _vph  || _vph <= -3.1415f*0.66f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_left != FLOOR_DETECTED){
          if(+3.1415f*0.16f < _vph  && _vph <= +3.1415f*0.84f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_right != FLOOR_DETECTED){
          if(-3.1415f*0.84f < _vph  && _vph <= -3.1415f*0.16f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        // 斜め
        if(_st_flrDtct.u8_rBack != FLOOR_DETECTED){
          if(+3.1415f*0.92f < _vph || _vph <= -3.1415f*0.42f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_rForward != FLOOR_DETECTED){
          if(-3.1415f*0.58f < _vph  && _vph <= +3.1415f*0.08f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_lForward != FLOOR_DETECTED){
          if(-3.1415f*0.08f < _vph  && _vph <= +3.1415f*0.58f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_lBack != FLOOR_DETECTED){
          if(+3.1415f*0.42f < _vph || _vph <= -3.1415f*0.92f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
      }

    }
#endif

    /* コマンド送信要求がある場合は送信 */
    if(_exist_tx_msg) {
      U32_MCN_NO_CMD_CNT   = 0;
      VDT::send_req_msg(&vdt_msg);
    } else {
      U32_MCN_NO_CMD_CNT++;
    }

    /* 何もコマンドが来ない場合、規定サイクル経過でSTOP指示 */
    if(U32_MCN_NO_CMD_CNT > U32_MCN_NO_CMD_STOP_THRE) {
      vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
      vdt_msg.move_dir.u32_cmd     = VDT::REQ_MOVE_DIR_CMD::MOVE_STOP;
      vdt_msg.move_dir.u32_time_ms = 1;
      vdt_msg.move_dir.u32_speed   = 0;
      U32_MCN_NO_CMD_CNT           = 0;
      VDT::send_req_msg(&vdt_msg);
    }

    /* Publish */
#if 1
    if(U8_PUB_PHASE == 0){
      U8_PUB_PHASE = 1;
      // Vehicle Info
      float px, py, pth;
      float vx, vy, vth;
      VDT::get_status_now_vehicle_pos_world(px, py, pth);
      VDT::get_status_now_vehicle_vel(vx, vy, vth);
      msg_pb_vhclInfo.pos.x = (int32_t)(px*1000.0f);  // [mm]
      msg_pb_vhclInfo.pos.y = (int32_t)(py*1000.0f);  // [mm]
      msg_pb_vhclInfo.pos.theta = pth;  // [rad]

      msg_pb_vhclInfo.vel.x = (int32_t)vx;  // [mm/s]
      msg_pb_vhclInfo.vel.y = (int32_t)vy;  // [mm/s]
      msg_pb_vhclInfo.vel.theta = vth;      // [rad/s]

      msg_pb_vhclInfo.floor.forward      = _st_flrDtct.u8_forward ;
      msg_pb_vhclInfo.floor.back         = _st_flrDtct.u8_back    ;
      msg_pb_vhclInfo.floor.right        = _st_flrDtct.u8_right   ;
      msg_pb_vhclInfo.floor.left         = _st_flrDtct.u8_left    ;
      msg_pb_vhclInfo.floor.rightforward = _st_flrDtct.u8_rForward;
      msg_pb_vhclInfo.floor.leftforward  = _st_flrDtct.u8_lForward;
      msg_pb_vhclInfo.floor.rightback    = _st_flrDtct.u8_rBack   ;
      msg_pb_vhclInfo.floor.leftback     = _st_flrDtct.u8_lBack   ;


      IMT::imu_data imudata = {};
      IMT::get_status_now_imu(imudata);
      if(imudata.is_error){
        msg_pb_vhclInfo.imu.fault = 0xFF;
        msg_pb_vhclInfo.imu.qx = 0;
        msg_pb_vhclInfo.imu.qy = 0;
        msg_pb_vhclInfo.imu.qz = 0;
        msg_pb_vhclInfo.imu.qw = 0;
        msg_pb_vhclInfo.imu.gx = 0;
        msg_pb_vhclInfo.imu.gy = 0;
        msg_pb_vhclInfo.imu.gz = 0;
        msg_pb_vhclInfo.imu.ax = 0;
        msg_pb_vhclInfo.imu.ay = 0;
        msg_pb_vhclInfo.imu.az = 0;
      }else{
        msg_pb_vhclInfo.imu.fault = 0;
        msg_pb_vhclInfo.imu.qx = imudata.qut[0];
        msg_pb_vhclInfo.imu.qy = imudata.qut[1];
        msg_pb_vhclInfo.imu.qz = imudata.qut[2];
        msg_pb_vhclInfo.imu.qw = imudata.qut[3];
        msg_pb_vhclInfo.imu.gx = imudata.gyr[0];
        msg_pb_vhclInfo.imu.gy = imudata.gyr[1];
        msg_pb_vhclInfo.imu.gz = imudata.gyr[2];
        msg_pb_vhclInfo.imu.ax = imudata.acc[0];
        msg_pb_vhclInfo.imu.ay = imudata.acc[1];
        msg_pb_vhclInfo.imu.az = imudata.acc[2];
      }

      msg_pb_vhclInfo.cam_pitch = CGT::get_pitch_angle_deg();
      msg_pb_vhclInfo.fault = (uint32_t)vdt_abort.val;
      rc = rcl_publish(&pb_vchlInfo, &msg_pb_vhclInfo, NULL);
      if(rc != RCL_RET_OK) return rc;
    } else {
      U8_PUB_PHASE = 0;
      // Arm Info
      float _fl_now_ang[5] = {};
      ADT::get_arm_angle_rad(_fl_now_ang);
      msg_pb_armInfo.servo.theta.data[0] = _fl_now_ang[0];
      msg_pb_armInfo.servo.theta.data[1] = _fl_now_ang[1];
      msg_pb_armInfo.servo.theta.data[2] = _fl_now_ang[2];
      msg_pb_armInfo.servo.theta.data[3] = _fl_now_ang[3];
      msg_pb_armInfo.servo.theta.data[4] = _fl_now_ang[4];
      msg_pb_armInfo.servo.theta.size = 5;

      rc = rcl_publish(&pb_ArmInfo, &msg_pb_armInfo, NULL);
      if(rc != RCL_RET_OK) return rc;
    }

#endif

  return RCL_RET_OK;
}


void prepare_task() {
  get_dev_mac_addr(mac_addr);
}

void main(void *params) {
  const TickType_t loop_tick = pdMS_TO_TICKS(1000 / 60);
  const TickType_t retry_tick = pdMS_TO_TICKS(500);
  TickType_t last_retry = xTaskGetTickCount() - retry_tick;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(RMT_MAIN);

    switch (UROS_AGENT_STATUS)
    {
    case ETHERNET_INIT:
      if((xTaskGetTickCount() - last_retry) >= retry_tick) {
        last_retry = xTaskGetTickCount();
#ifdef USE_HOME_NETWORK
        if(!Ethernet.begin(mac_addr, 1000, 100)) break;
#else
        Ethernet.begin(mac_addr, device_ip);
#endif
        DEBUG_PRINT_RMT("[RMT]IP%x\n", uint32_t(Ethernet.localIP()));
        set_microros_native_ethernet_udp_transports((byte *)mac_addr, device_ip, agent_ip, agent_port);
        UROS_AGENT_STATUS = WAITING_AGENT;
      }
      break;
    case WAITING_AGENT:
      /* 切断後のAgentからのPing応答待ち状態 */
      UTIL::set_LED1(true);
      if((xTaskGetTickCount() - last_retry) >= retry_tick) {
        last_retry = xTaskGetTickCount();
        if(RMW_RET_OK == rmw_uros_ping_agent(20, 1)){
          UROS_AGENT_STATUS = AVAILABLE_AGENT;
        }
      }
      break;
    case AVAILABLE_AGENT:
      DEBUG_PRINT_STR_RMT("[RMT]Recreate uros entities\n");
      if(create_microros_entities() == RCL_RET_OK) {
        UROS_AGENT_STATUS = CONNECTED;
      } else {
        UROS_AGENT_STATUS = WAITING_AGENT;
        last_retry = xTaskGetTickCount();
      }
      break;
    case CONNECTED:
      if(U32_UROS_PING_COUNTER >= U32_UROS_PING_COUNTER_MATCH){
        U32_UROS_PING_COUNTER = 0;
        if(RMW_RET_OK == rmw_uros_ping_agent(20, 2)){
          const rcl_ret_t rc = routine_ros();
          if(rc != RCL_RET_OK) {
            report_rcl_failure("micro-ROS routine", rc);
            UROS_AGENT_STATUS = DISCONNECTED;
            break;
          }
          UTIL::set_LED1(false);
        } else {
          UROS_AGENT_STATUS = DISCONNECTED;
        }
      } else {
        const rcl_ret_t rc = routine_ros();
        if(rc != RCL_RET_OK) {
          report_rcl_failure("micro-ROS routine", rc);
          UROS_AGENT_STATUS = DISCONNECTED;
          break;
        }
        UTIL::set_LED1(false);
        U32_UROS_PING_COUNTER++;
      }
      break;
    case DISCONNECTED:
      DEBUG_PRINT_STR_RMT("[RMT]destroy uros entities\n");
      destroy_microros_entities();
      UROS_AGENT_STATUS = WAITING_AGENT;
      break;
    default:
      break;
    }

    DEBUG_PRINT_PRC_FINISH(RMT_MAIN);
  }
}

} // namespace RMT
