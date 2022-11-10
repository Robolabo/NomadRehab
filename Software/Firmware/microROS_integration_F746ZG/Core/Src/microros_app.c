/*
 * microro_app.c
 *
 *  Created on: Oct 10, 2022
 *      Author: alejo
 */

#include "microros_app.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define TELEMETRY_TOPIC "/control/telemetry"
#define LED_TOPIC "/control/led"
#define NODE_NAME  "telemetry_node"

rcl_publisher_t telemetry_pub;
rcl_subscription_t led_enable_sub;
rcl_timer_t pub_timer;


sensor_telemetry__msg__Telemetry outcomming_telemetry;
std_msgs__msg__Bool inconming_led;


void led_subscription_callback(const void* data) {
	const std_msgs__msg__Bool* msg = (std_msgs__msg__Bool*)data;

	if (msg->data == 1) {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	} else {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	outcomming_telemetry.temperature += 0.5;
	outcomming_telemetry.humidity += 0.5;

	if (outcomming_telemetry.temperature > 1000.0) {
	  outcomming_telemetry.temperature = 0;
	}
  if (outcomming_telemetry.humidity > 1000.0) {
    outcomming_telemetry.temperature = 0;
  }

	if (timer != NULL) {
		rcl_publish(&telemetry_pub, &outcomming_telemetry, NULL);
	}
}





rclc_support_t* support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

void main_ros_init(rclc_support_t* uros_support) {
  support = (rclc_support_t*)(uros_support);
  allocator = rcl_get_default_allocator();

  // create node

  RCCHECK(rclc_node_init_default(&node, "add_twoints_server_rclc", "", support));


  rcl_ret_t ret = 0;
  /* Create publisher */
  ret = rclc_publisher_init_default(
      &telemetry_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_telemetry, msg, Telemetry),
      TELEMETRY_TOPIC);

  /* Create subscriber */

  ret = rclc_subscription_init_default(
      &led_enable_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      LED_TOPIC);


  /* Create timer */
  ret = rclc_timer_init_default(&pub_timer, support, RCL_MS_TO_NS(1), timer_callback);

  ret = rclc_executor_init(&executor, &(support->context), 3, &allocator);

  ret = rclc_executor_add_subscription(
      &executor,
      &led_enable_sub,
      &inconming_led,
      led_subscription_callback,
      ON_NEW_DATA);

  ret = rclc_executor_add_timer(&executor, &pub_timer);
}



void main_ros_app(void* params) {
	while(1){

		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
		vTaskDelay(1);
	}
}

void create_ros_task(rclc_support_t* uros_support) {

  main_ros_init(uros_support);
	xTaskCreate(
			main_ros_app,
			ROS_TASK_NAME,
			ROS_TASK_STACK,
			NULL,
			ROS_TASK_PRIO,
			NULL);
}
