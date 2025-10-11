#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "usart.h"
#include "gpio.h"

#include "robot.h"


#ifndef ROBOT_ID
#define ROBOT_ID -1
#endif


typedef StaticTask_t osStaticThreadDef_t;

osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[500];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void StartPublisherTask(void *argument);
void StartAnotherPublisherTask(void *argument);

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void) {
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart3,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }
}


const unsigned int timer_period_10_ms = RCL_MS_TO_NS(10);
const unsigned int timer_period_25_ms = RCL_MS_TO_NS(25);

rcl_subscription_t subscriber_cmd_vel_l;
rcl_subscription_t subscriber_cmd_vel_r;
rcl_subscription_t subscriber_reset_motors;
rcl_publisher_t publisher_cmd_vel_l;
rcl_publisher_t publisher_cmd_vel_r;
rcl_publisher_t publisher_ticks_r;
rcl_publisher_t publisher_ticks_l;

rcl_timer_t cmd_vel_timer, odom_timer;

float wl;


void cmdVelCallbackLeft(const void* msgin) {
	const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
    wl = msg->data;
}


void cmdVelCallbackRight(const void* msgin) {
	const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
    cmd_vel_callback(wl, msg->data);
}


void resetMotorsCallback(const void* msgin) {
    resetMotors();
}


void cmdVelTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    spinMotors();
    std_msgs__msg__Float32 msg;
    msg.data = get_vel_l();
    rcl_publish(&publisher_cmd_vel_l, &msg, NULL);
    msg.data = get_vel_r();
    rcl_publish(&publisher_cmd_vel_r, &msg, NULL);
}


void odomTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    std_msgs__msg__Int64 msg;
    msg.data = get_ticks_l();
    rcl_publish(&publisher_ticks_l, &msg, NULL);
    msg.data = get_ticks_r();
    rcl_publish(&publisher_ticks_r, &msg, NULL);
}


void StartDefaultTask(void* argument) {
    wl = 0.0;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

	rclc_support_init(&support, 0, NULL, &allocator);

	rcl_node_t node;
    char namespace[32];
    snprintf(namespace, sizeof(namespace), "robot%d", ROBOT_ID);
	rclc_node_init_default(&node, "stm32_node", namespace, &support);

    rclc_timer_init_default(&cmd_vel_timer, &support, timer_period_25_ms, cmdVelTimerCallback);
    rclc_timer_init_default(&odom_timer, &support, timer_period_10_ms, odomTimerCallback);

	rclc_subscription_init_default(&subscriber_cmd_vel_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drive_controller/cmd_vel_l");
	rclc_subscription_init_default(&subscriber_cmd_vel_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drive_controller/cmd_vel_r");
    rclc_subscription_init_default(&subscriber_reset_motors, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "drive_controller/reset");

    rclc_publisher_init_default(&publisher_cmd_vel_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drive_controller/cmd_vel_back_l");
    rclc_publisher_init_default(&publisher_cmd_vel_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drive_controller/cmd_vel_back_r");
    rclc_publisher_init_default(&publisher_ticks_l, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "drive_controller/ticks_left");
    rclc_publisher_init_default(&publisher_ticks_r, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64), "drive_controller/ticks_right");

	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 6, &allocator);

    std_msgs__msg__Float32 sub_msg_cmd_vel_l;
    std_msgs__msg__Float32 sub_msg_cmd_vel_r;
    std_msgs__msg__Float32 sub_msg_reset;

    rclc_executor_add_subscription(&executor, &subscriber_cmd_vel_l, &sub_msg_cmd_vel_l, &cmdVelCallbackLeft, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriber_cmd_vel_r, &sub_msg_cmd_vel_r, &cmdVelCallbackRight, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriber_reset_motors, &sub_msg_reset, &resetMotorsCallback, ON_NEW_DATA);
	
    rclc_executor_add_timer(&executor, &cmd_vel_timer);
    rclc_executor_add_timer(&executor, &odom_timer);

    rclc_executor_spin(&executor);

	rcl_subscription_fini(&subscriber_cmd_vel_l, &node);
	rcl_subscription_fini(&subscriber_cmd_vel_r, &node);
	rcl_subscription_fini(&subscriber_reset_motors, &node);
    rcl_timer_fini(&cmd_vel_timer);
    rcl_timer_fini(&odom_timer);
	rcl_node_fini(&node);

    while(1) {

    }
}
