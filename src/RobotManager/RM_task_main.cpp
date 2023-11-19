// RTOS
#include <FreeRTOS_TEENSY4.h>

#include <geometry_msgs/msg/twist.h>
#include <interfaces/msg/arm_info.h>
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
#include <stdio.h>

#include "../ArmDrive/AD_task_main.hpp"
#include "../CameraGimbal/CG_task_main.hpp"
#include "../FloorDetect/FD_task_main.hpp"
#include "../Utility/util_led.hpp"
#include "../Utility/util_mymath.hpp"
#include "../VehicleDrive/VD_task_main.hpp"

// Local status
/* Ethernet */
bool is_ethernet_init_successful = false;
bool is_microros_init_successful = false;
enum ConnectionStatus{
  WAITING_AGENT,
  AVAILABLE_AGENT,
  CONNECTED,
  DISCONNECTED,
  UNKNOWN,
};
ConnectionStatus UROS_AGENT_STATUS = WAITING_AGENT;
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
interfaces__msg__VehicleInfo msg_pb_vhclInfo;
interfaces__msg__ArmInfo     msg_pb_armInfo;

uint8_t U8_PUB_PHASE = 0;

// Buffer for ArmInfo
static constexpr uint8_t U8_ARMANGLE_BUF_LEN = 5;
float fl_ArmAngThetaBuffer[U8_ARMANGLE_BUF_LEN] = {};

// subscriber
rcl_subscription_t              sb_mcnmCmd;
rcl_subscription_t              sb_mcnmContOdr;
rcl_subscription_t              sb_tmAngle;
rcl_subscription_t              sb_cmd;
interfaces__msg__MecanumCommand msg_sb_mcnmCmd;
interfaces__msg__MecanumContOrder msg_sb_mcnmContOdr;
interfaces__msg__TimeAngle      msg_sb_tmAngle;
interfaces__msg__Command        msg_sb_cmd;

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

#define RCCHECK(fn)                               \
  {                                               \
    rcl_ret_t temp_rc = fn;                       \
    if((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if((temp_rc != RCL_RET_OK)) {} \
  }

#define toggleRMtaskLED UTIL::toggle_LED1

void error_loop() {
  while(1) {
    toggleRMtaskLED();
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if(timer != NULL) {
    RCSOFTCHECK(rcl_publish(&pb_vchlInfo, &msg_pb_vhclInfo, NULL));
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
  switch(msg->command) {
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
  case CmdStatus::QUIT_PG:
  default: {
    /* Arm角度初期化(キャリブレーション) */
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId            = ADT::MSG_ID::REQ_CHANGE_MODE;
    adt_msg.change_mode.u32_mode_id = ADT::MODE_ID::OFF;
    adt_msg.change_mode.u8_forced   = 1;
    ADT::send_req_msg(&adt_msg);
  }
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

void sb_timeAngle_callback(const void *msgin) {
  const interfaces__msg__TimeAngle *msg = (const interfaces__msg__TimeAngle *)msgin;

  DEBUG_PRINT_STR_RMT("[RMT]TimeAngRecv\n");

  /* 二重受け取り防止用処理 */
  const uint32_t CU32_TANG_ID_NO_DATA = 99;
  uint32_t       _u32_sts             = ADT::get_status_timeangle_proc(msg->id);
  if(_u32_sts == CU32_TANG_ID_NO_DATA) {
    // 受け取ったデータがキューのどこにも存在しない場合のみ受け付ける
    ADT::MSG_REQ adt_msg;
    adt_msg.common.MsgId        = ADT::MSG_ID::REQ_MOVE_TIMEANGLE;
    adt_msg.time_angle.u32_id   = msg->id;
    adt_msg.time_angle.u32_len  = msg->arm[0].point.size;
    adt_msg.time_angle.ptr_tAng = (interfaces__msg__TimeAngle *)msg;

    ADT::send_req_msg(&adt_msg);
  } else {
    // 処理中or処理後の場合は、二重受け取りなので破棄
    DEBUG_PRINT_STR_RMT("[RMT]TimeAng overlap\n");
  }

  DEBUG_PRINT_STR_RMT("[RMT]TimeAngCplt\n");
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
#ifdef USE_HOME_NETWORK
IPAddress device_ip(192, 168, 10, 177);
// IPAddress agent_ip(192, 168, 10, 128);  // Jetson
// IPAddress agent_ip(192, 168, 10, 117); // laptop
IPAddress agent_ip(192, 168, 10, 110); // desktop
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

static void create_microros_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "quinque_hw", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &pb_vchlInfo,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, VehicleInfo),
      "VehicleInfo"));

  RCCHECK(rclc_publisher_init_best_effort(
      &pb_ArmInfo,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, ArmInfo),
      "ArmInfo"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &sb_mcnmCmd,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, MecanumCommand),
      "MecanumCommand"));

  /* なぜかめちゃくちゃメモリ食う。しかもpaddingが31kbyteもできる。要検証 */
  RCCHECK(rclc_subscription_init_default(
      &sb_mcnmContOdr,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, MecanumContOrder),
      "MecanumContOrder"));

  RCCHECK(rclc_subscription_init_default(
      &sb_tmAngle,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, TimeAngle),
      "TimeAngle"));

  RCCHECK(rclc_subscription_init_default(
      &sb_cmd,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, Command),
      "Command"));

  // create service
  RCCHECK(rclc_service_init_default(
      &srv_proc,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(interfaces, srv, ProcStatus),
      "ProcStatus"));

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
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_mcnmCmd, &msg_sb_mcnmCmd, &sb_mecanumCmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_mcnmContOdr, &msg_sb_mcnmContOdr, &sb_mecanumContOdr_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_tmAngle, &msg_sb_tmAngle, &sb_timeAngle_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_cmd, &msg_sb_cmd, &sb_cmd_callback, ON_NEW_DATA));

  executor_srv = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_srv, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor_srv, &srv_proc, &srv_req_procSts, &srv_res_procSts, srv_procSts_callback));

  /* 可変長MessageのBufferを設定 */
  for(int i = 0; i < 5; i++) {
    msg_sb_tmAngle.arm[i].point.data     = TimeAngleBuffer[i];
    msg_sb_tmAngle.arm[i].point.capacity = U8_TIMEANGLE_BUF_LEN;
  }

  msg_pb_armInfo.servo.theta.data     = &fl_ArmAngThetaBuffer[0];
  msg_pb_armInfo.servo.theta.capacity = U8_ARMANGLE_BUF_LEN;

  is_microros_init_successful = true;

  DEBUG_PRINT_STR_RMT("[RMT] Micro-ROS initialization complete\n");
}

static void destroy_microros_entities(){
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&pb_vchlInfo, &node));
  RCSOFTCHECK(rcl_publisher_fini(&pb_ArmInfo, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sb_mcnmCmd, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sb_mcnmContOdr, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sb_tmAngle, &node));
  RCSOFTCHECK(rcl_subscription_fini(&sb_cmd, &node));
  RCSOFTCHECK(rcl_service_fini(&srv_proc, &node));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rclc_executor_fini(&executor_srv));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

static void routine_ros(){
    /* Micro-ROS接続完了時のRoutine */

    /* Spin 処理 */
    rclc_executor_spin_some(&executor, RCUTILS_US_TO_NS(2000));     // Subscribe
    rclc_executor_spin_some(&executor_srv, RCUTILS_US_TO_NS(1000)); // Service

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
          if(-3.1415f*0.25f < _vph  && _vph <= +3.1415f*0.25f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_back != FLOOR_DETECTED){
          if(+3.1415f*0.75f < _vph  && _vph <= -3.1415f*0.75f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_left != FLOOR_DETECTED){
          if(+3.1415f*0.25f < _vph  && _vph <= +3.1415f*0.75f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_right != FLOOR_DETECTED){
          if(-3.1415f*0.75f < _vph  && _vph <= -3.1415f*0.25f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        // 斜め
        if(_st_flrDtct.u8_rBack != FLOOR_DETECTED){
          if(_vph <= -3.1415f*0.5f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_rForward != FLOOR_DETECTED){
          if(-3.1415f*0.5f < _vph  && _vph <= 0.0f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_lForward != FLOOR_DETECTED){
          if(0.0f < _vph  && _vph <= +3.1415f*0.5f){
              vdt_msg.move_cont_dir.fl_vel_x_mmps = 0;
              vdt_msg.move_cont_dir.fl_vel_y_mmps = 0;
              vdt_abort.bit.fllr_abort_vdt_cont_trans_dir = 1;
          }else{
          }
        }
        if(_st_flrDtct.u8_lBack != FLOOR_DETECTED){
          if(_vph <= +3.1415f*0.5f){
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
      float vx, vy, vth;
      VDT::get_status_now_vehicle_vel(vx, vy, vth);
      msg_pb_vhclInfo.pos.x = (int32_t)vx;
      msg_pb_vhclInfo.pos.y = (int32_t)vy;
      msg_pb_vhclInfo.pos.theta = vth;

      msg_pb_vhclInfo.floor.forward      = _st_flrDtct.u8_forward ;
      msg_pb_vhclInfo.floor.back         = _st_flrDtct.u8_back    ;
      msg_pb_vhclInfo.floor.right        = _st_flrDtct.u8_right   ;
      msg_pb_vhclInfo.floor.left         = _st_flrDtct.u8_left    ;
      msg_pb_vhclInfo.floor.rightforward = _st_flrDtct.u8_rForward;
      msg_pb_vhclInfo.floor.leftforward  = _st_flrDtct.u8_lForward;
      msg_pb_vhclInfo.floor.rightback    = _st_flrDtct.u8_rBack   ;
      msg_pb_vhclInfo.floor.leftback     = _st_flrDtct.u8_lBack   ;

      msg_pb_vhclInfo.cam_pitch = CGT::get_pitch_angle_deg();
      msg_pb_vhclInfo.fault = (uint32_t)vdt_abort.val;
      RCSOFTCHECK(rcl_publish(&pb_vchlInfo, &msg_pb_vhclInfo, NULL));
    } else if(U8_PUB_PHASE == 1){
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

      RCSOFTCHECK(rcl_publish(&pb_ArmInfo, &msg_pb_armInfo, NULL));
    }

#endif

}


void prepare_task() {
  get_dev_mac_addr(mac_addr);

  /* EtherNet接続 */
  while(!is_ethernet_init_successful) {
#ifdef USE_HOME_NETWORK
    is_ethernet_init_successful = Ethernet.begin(mac_addr, 1000, 100);
#else
    Ethernet.begin(mac_addr, device_ip);
    is_ethernet_init_successful = true;
#endif
    // vTaskDelay(500);
    delay(500);
  }

  DEBUG_PRINT_RMT("[RMT]IP%x\n", uint32_t(Ethernet.localIP()));

  /* EtherNetに繋がった場合、Micro-ROSに接続を試みる */
  set_microros_native_ethernet_udp_transports((byte *)mac_addr, device_ip, agent_ip, 9999);

  while(!is_microros_init_successful) {
    if(RMW_RET_OK == rmw_uros_ping_agent(50, 2)) {
      create_microros_entities();
    } else {
      // 何もしない
    }
    // vTaskDelay(500);
    delay(500);
  }

  delay(500);

  UROS_AGENT_STATUS = CONNECTED;
}

void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / 60;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(RMT_MAIN);

    switch (UROS_AGENT_STATUS)
    {
    case WAITING_AGENT:
      /* 切断後のAgentからのPing応答待ち状態 */
      DEBUG_PRINT_STR_RMT("[RMT]waiting uros agent response\n");
      UTIL::set_LED1(true);
      set_microros_native_ethernet_udp_transports((byte *)mac_addr, device_ip, agent_ip, 9999);
      if(RMW_RET_OK == rmw_uros_ping_agent(20, 1)){
        UROS_AGENT_STATUS = AVAILABLE_AGENT;
      }
      break;
    case AVAILABLE_AGENT:
      DEBUG_PRINT_STR_RMT("[RMT]Recreate uros entities\n");
      create_microros_entities();
      UROS_AGENT_STATUS = CONNECTED;
      break;
    case CONNECTED:
      if(U32_UROS_PING_COUNTER >= U32_UROS_PING_COUNTER_MATCH){
        U32_UROS_PING_COUNTER = 0;
        if(RMW_RET_OK == rmw_uros_ping_agent(20, 2)){
          routine_ros();
          UTIL::set_LED1(false);
        } else {
          UROS_AGENT_STATUS = DISCONNECTED;
        }
      } else {
          routine_ros();
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