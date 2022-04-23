

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <test_msg/msg/robot_info.h>

#include "../Utility/util_led.hpp"

rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
test_msg__msg__RobotInfo msg_roboinfo;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define toggleRMtaskLED UTIL::toggle_LED0

void error_loop(){
  while(1){
    toggleRMtaskLED();
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_roboinfo, NULL));
    msg_roboinfo.pos_x++;
    msg_roboinfo.pos_y--;
    msg_roboinfo.theta++;
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  toggleRMtaskLED();
  Serial.printf("li x:%d, y:%d, z:%d\n", static_cast<int>(msg->linear.x), 
                                         static_cast<int>(msg->linear.y),
                                         static_cast<int>(msg->linear.z)); 
  Serial.printf("an x:%d, y:%d, z:%d\n", static_cast<int>(msg->angular.x), 
                                         static_cast<int>(msg->angular.y),
                                         static_cast<int>(msg->angular.z)); 
}

void setup_microros_test(){
  byte arduino_mac[] = { 0x04, 0xE9, 0xE5, 0x11, 0x97, 0x8B };
  IPAddress arduino_ip(192, 168, 10, 177);
  IPAddress agent_ip(192, 168, 10, 117);
  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);

  delay(2000);
  uint8_t mac[6] = {};
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_ethernet_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(test_msg, msg, RobotInfo),
    "topic_name"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "turtle1/cmd_vel"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(10);
	}

  //RCCHECK(rcl_subscription_fini(&subscriber, &node));
  //RCCHECK(rcl_node_fini(&node));
}