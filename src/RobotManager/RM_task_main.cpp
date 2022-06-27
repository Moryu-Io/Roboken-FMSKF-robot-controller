// RTOS
#include <FreeRTOS_TEENSY4.h>

#include <geometry_msgs/msg/twist.h>
#include <interfaces/msg/arm_info.h>
#include <interfaces/msg/mecanum_command.h>
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
#include "../Utility/util_led.hpp"
#include "../VehicleDrive/VD_task_main.hpp"

bool is_ethernet_init_successful = false;
bool is_microros_init_successful = false;

// publisher
rcl_publisher_t              pb_vchlInfo;
rcl_publisher_t              pb_ArmInfo;
interfaces__msg__VehicleInfo msg_pb_vhclInfo;
interfaces__msg__ArmInfo     msg_pb_armInfo;

// subscriber
rcl_subscription_t              sb_mcnmCmd;
rcl_subscription_t              sb_tmAngle;
interfaces__msg__MecanumCommand msg_sb_mcnmCmd;
interfaces__msg__TimeAngle      msg_sb_tmAngle;

// Buffer for TimeAngle
static constexpr uint8_t U8_TIMEANGLE_BUF_LEN                     = 16;
interfaces__msg__Joint   TimeAngleBuffer[5][U8_TIMEANGLE_BUF_LEN] = {};

// service
rcl_service_t                        srv_proc;
interfaces__srv__ProcStatus_Request  srv_req_procSts;
interfaces__srv__ProcStatus_Response srv_res_procSts;

rclc_executor_t executor;

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

void sb_mecanumCmd_callback(const void *msgin) {
  const interfaces__msg__MecanumCommand *msg = (const interfaces__msg__MecanumCommand *)msgin;

  VDT::MSG_REQ vdt_msg;
  vdt_msg.common.MsgId         = VDT::MSG_ID::REQ_MOVE_DIR;
  vdt_msg.move_dir.u32_cmd     = msg->cmd;
  vdt_msg.move_dir.u32_time_ms = msg->time;
  vdt_msg.move_dir.u32_speed   = msg->speed;

  VDT::send_req_msg(&vdt_msg);

  DEBUG_PRINT_STR_RMT("[RMT]McnmCmd\n");
}

void sb_timeAngle_callback(const void *msgin) {
  const interfaces__msg__TimeAngle *msg = (const interfaces__msg__TimeAngle *)msgin;

  ADT::MSG_REQ adt_msg;
  adt_msg.common.MsgId        = ADT::MSG_ID::REQ_MOVE_TIMEANGLE;
  adt_msg.time_angle.u32_id   = msg->id;
  adt_msg.time_angle.u32_len  = msg->arm[0].point.size;
  adt_msg.time_angle.ptr_tAng = (interfaces__msg__TimeAngle *)msg;

  ADT::send_req_msg(&adt_msg);

  DEBUG_PRINT_STR_RMT("[RMT]TimeAng\n");
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
//IPAddress agent_ip(192, 168, 10, 128);  // Jetson
IPAddress agent_ip(192, 168, 10, 117);
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
  RCCHECK(rclc_subscription_init_best_effort(
      &sb_mcnmCmd,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, MecanumCommand),
      "MecanumCommand"));

  RCCHECK(rclc_subscription_init_best_effort(
      &sb_tmAngle,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(interfaces, msg, TimeAngle),
      "TimeAngle"));

  // create service
  RCCHECK(rclc_service_init_best_effort(
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_mcnmCmd, &msg_sb_mcnmCmd, &sb_mecanumCmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sb_tmAngle, &msg_sb_tmAngle, &sb_timeAngle_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(&executor, &srv_proc, &srv_req_procSts, &srv_res_procSts, srv_procSts_callback));

  /* 可変長MessageのBufferを設定 */
  for(int i = 0; i < 5; i++) {
    msg_sb_tmAngle.arm[i].point.data     = TimeAngleBuffer[i];
    msg_sb_tmAngle.arm[i].point.capacity = U8_TIMEANGLE_BUF_LEN;
  }

  is_microros_init_successful = true;

  DEBUG_PRINT_STR_RMT("[RMT] Micro-ROS initialization complete\n");
}

void prepare_task() {
  get_dev_mac_addr(mac_addr);
}

void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / 120;

  /* EtherNet接続 */
  while(!is_ethernet_init_successful) {
#ifdef USE_HOME_NETWORK
    is_ethernet_init_successful = Ethernet.begin(mac_addr, 1000, 100);
#else
    Ethernet.begin(mac_addr, device_ip);
    is_ethernet_init_successful = true;
#endif
    vTaskDelay(500);
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
    vTaskDelay(500);
  }

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);

    /* Micro-ROS接続完了時のRoutine */
    // msg_pb_vhclInfo.pos.x++;
    // msg_pb_vhclInfo.pos.y--;
    // msg_pb_vhclInfo.pos.theta += 0.1f;
    // RCSOFTCHECK(rcl_publish(&pb_vchlInfo, &msg_pb_vhclInfo, NULL));

    /* ROS 処理 */
    rclc_executor_spin_some(&executor, RCUTILS_US_TO_NS(1000));

    /* 車体Manage処理 */
    
  }
}

} // namespace RMT