/*
 * microro_app.c
 *
 *  Created on: Oct 10, 2022
 *      Author: alejo
 */

#include "microros_app.h"


#define UROS_MOTOR_INDEX(index)\
  (index) == 0U ? NOMAD_PWM_CHANNEL_1:\
  (index) == 1U ? NOMAD_PWM_CHANNEL_2:\
  (index) == 2U ? NOMAD_PWM_CHANNEL_3:\
  0xFFFFFFFFU


typedef motor_msgs__msg__MotorData MotorDutyData_t;
typedef motor_msgs__msg__MotorPosition MotorPostion_t;
typedef motor_msgs__srv__MotorService_Request MotorDutyRequest_t;
typedef motor_msgs__srv__MotorService_Response MotorDutyResponse_t;


static rcl_subscription_t UROS_duty_subscription;
static rcl_subscription_t UROS_position_subscription;
static rcl_publisher_t UROS_position_publisher;
static rcl_timer_t UROS_position_timer;


/* ToDo: Define posible controller */

#if 0
static rcl_subscription_t UROS_target_speed_subscription;
static rcl_subscription_t UROS_target_position_subscription;
static rcl_subscription_t UROS_controller_config;
#endif

static rcl_service_t UROS_get_duty_service;
static rcl_node_t UROS_node;

static MotorDutyData_t UROS_motor_duty_buffer;
static MotorDutyRequest_t UROS_motor_duty_srv_request;
static MotorDutyResponse_t UROS_motor_duty_srv_response;
static MotorPostion_t UROS_motor_position_rx_buffer;
static MotorPostion_t UROS_motor_position_tx_buffer;


static void UROS_motor_duty_callback(const void* data) {
	const MotorDutyData_t* msg = (MotorDutyData_t*)data;

	/* Set duty cycle */
	NOMAD_PWM_setDuty(UROS_MOTOR_INDEX(msg->index), msg->duty);
}

static void UROS_position_callback(const void* data) {
  RCL_UNUSED(data);
  /* Reset position */
}


void UROS_position_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  /* Get current position */
  UROS_motor_position_tx_buffer.pos_motor_1 = 0.0;
  UROS_motor_position_tx_buffer.pos_motor_3 = 0.0;
  UROS_motor_position_tx_buffer.pos_motor_2 = 0.0;

  if (timer != NULL) {
    rcl_publish(&UROS_position_publisher, &UROS_motor_position_tx_buffer, NULL);
  }
}

static void UROS_motor_duty_srv_callback(const void* request_data, void* response_data) {
  const MotorDutyRequest_t* request = (MotorDutyRequest_t*)request_data;
  MotorDutyResponse_t* response = (MotorDutyResponse_t*)response_data;

  response->index = request->index;
  response->duty = NOMAD_PWM_get_duty_cycle(UROS_MOTOR_INDEX(request->index));
}

static void UROS_init_transports() {

	rmw_uros_set_custom_transport(
	true,
	(void *) &huart2,
	transport_serial_open,
	transport_serial_close,
	transport_serial_write,
	transport_serial_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = __freertos_allocate;
  freeRTOS_allocator.deallocate = __freertos_deallocate;
  freeRTOS_allocator.reallocate = __freertos_reallocate;
  freeRTOS_allocator.zero_allocate = __freertos_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator))
  {
    /* Something went wrong */
    while (1);
  }
}


void UROS_motor_app() {

  UROS_init_transports();

	rcl_ret_t ret= 0;

  rclc_executor_t executor;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	/* Init options */
	ret = rclc_support_init(&support, 0, NULL, &allocator);

	/* Init node with custom options */
	ret = rclc_node_init_default(&UROS_node, UROS_NODE_NAME, "", &support);

	/* Create subscriber */
	ret = rclc_subscription_init_default(
			&UROS_duty_subscription,
			&UROS_node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(motor_msgs, msg, MotorData),
			UROS_MOTOR_DUTY_TOPIC);

  ret = rclc_subscription_init_default(
      &UROS_position_subscription,
      &UROS_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(motor_msgs, msg, MotorPosition),
      UROS_MOTOR_POS_RX_TOPIC);

  ret = rclc_publisher_init_default(
      &UROS_position_publisher,
      &UROS_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(motor_msgs, msg, MotorPosition),
      UROS_MOTOR_POS_TX_TOPIC);

	ret = rclc_service_init_default(
	    &UROS_get_duty_service,
	    &UROS_node,
	    ROSIDL_GET_SRV_TYPE_SUPPORT(motor_msgs, srv, MotorService),
	    UROS_MOTOR_SERVICE);

  ret = rclc_timer_init_default(
      &UROS_position_timer,
      &support,
      RCL_MS_TO_NS(UROS_MOTOR_POS_TIME_MS),
      UROS_position_timer_callback);

	ret = rclc_executor_init(&executor, &support.context, UROS_MOTOR_N_INSTANCES, &allocator);

	ret = rclc_executor_add_subscription(
			&executor,
			&UROS_duty_subscription,
			&UROS_motor_duty_buffer,
			UROS_motor_duty_callback,
			ON_NEW_DATA);

  ret = rclc_executor_add_subscription(
      &executor,
      &UROS_position_subscription,
      &UROS_motor_position_rx_buffer,
      UROS_position_timer_callback,
      ON_NEW_DATA);

  ret = rclc_executor_add_timer(&executor, &UROS_position_timer);

	ret = rclc_executor_add_service(
	    &executor,
	    &UROS_get_duty_service,
	    &UROS_motor_duty_srv_request,
	    &UROS_motor_duty_srv_response,
	    UROS_motor_duty_srv_callback);

  /* Avoid unused warning */
  RCL_UNUSED(ret);

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(UROS_SPIN_TIME_MS));
		vTaskDelay(pdMS_TO_TICKS(UROS_DELAY_TIME_MS));
	}
}


void create_ros_task() {
	xTaskCreate(
	    UROS_motor_app,
			UROS_TASK_NAME,
			UROS_TASK_STACK,
			NULL,
			UROS_TASK_PRIO,
			NULL);
}
