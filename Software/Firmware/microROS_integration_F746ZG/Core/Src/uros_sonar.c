/*
 * uros_sonar.c
 *
 *  Created on: Oct 30, 2022
 *      Author: agome
 */


#include "uros_sonar.h"


/* Syntax sugar */

typedef sonar_msgs__srv__SonarService_Request SonarRequest_t;
typedef sonar_msgs__srv__SonarService_Response SonarResponse_t;
typedef sonar_msgs__msg__SonarData SonarSingle_t;

/* Variables */

static rcl_publisher_t UROS_sonar_publisher;
static rcl_service_t UROS_sonar_service;
static SonarRequest_t UROS_sonar_request;
static SonarResponse_t UROS_sonar_response;
static SonarSingle_t UROS_sonar_buffer;
static rcl_timer_t UROS_sonar_timer;



void UROS_sonar_service_callback(const void* request, void* response) {
  SonarRequest_t* sonar_request = (SonarRequest_t*)(request);
  SonarResponse_t* sonar_response = (SonarResponse_t*)(response);

  sonar_response->index = sonar_request->index;
  if (IS_CORRECT_SONAR_INDEX(sonar_request->index)) {
    sonar_response->distance = NOMAD_get_sonar_distance(sonar_request->index);
  } else {
    sonar_response->distance = 0.0;
  }
}


void UROS_periodic_sonar_callback(rcl_timer_t * timer, int64_t last_call_time) {
  static uint8_t index = 0;
  rcl_ret_t ret;
  RCLC_UNUSED(ret);
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    UROS_sonar_buffer.index = index;
    UROS_sonar_buffer.distance = NOMAD_get_sonar_distance(index);
    index = (index + 1) % 16U;
    /* Send data */
    ret = rcl_publish(&UROS_sonar_publisher, (const void*)(&UROS_sonar_buffer), NULL);
  }
}

static void UROS_sonar_app() {
  rcl_node_t node;
  rclc_support_t support;
  rclc_executor_t executor;
  rcl_allocator_t allocators = rcl_get_default_allocator();

  NOMAD_sonar_init();

  rclc_support_init(&support, 0, NULL, &allocators);
  rclc_node_init_default(&node, UROS_SONAR_NODE_NAME, UROS_SONAR_NAMESPACE, &support);
  rclc_executor_init(&executor, &support.context, UROS_SONAR_MAX_HANDLES, &allocators);

#if defined(UROS_SONAR_ENABLE_PERIODIC) && ((UROS_SONAR_ENABLE_PERIODIC) == 1)
  rclc_publisher_init_default(
      &UROS_sonar_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sonar_msgs, msg, SonarData),
      UROS_SONAR_TOPIC_NAME);

  rclc_timer_init_default(
      &UROS_sonar_timer,
      &support,
      RCL_MS_TO_NS(UROS_SONAR_PERIOD_MS),
      UROS_periodic_sonar_callback);

  rclc_executor_add_timer(&executor, &UROS_sonar_timer);
#endif

  rclc_service_init_default(
      &UROS_sonar_service,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(sonar_msgs, srv, SonarService),
      UROS_SONAR_SERVICE_NAME);

  rclc_executor_add_service(
      &executor,
      &UROS_sonar_service,
      (void*)(&UROS_sonar_request),
      (void*)(&UROS_sonar_response),
      UROS_sonar_service_callback);

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(UROS_SONAR_EXEC_PERIOD_MS));
    vTaskDelay(UROS_SONAR_EXEC_PERIOD_MS);
  }
}


void UROS_sonar_create_app() {
  xTaskCreate(
      UROS_sonar_app,
      UROS_SONAR_TASK_NAME,
      UROS_SONAR_TASK_STACK,
      NULL,
      UROS_SONAR_TASK_PRIO,
      NULL);
}
