/**                             _____________
 *              /\      /\     /             \
 *             //\\____//\\   |  TUlBVVVVVSE= |
 *            /     '      \   \  ___________/
 *           /   /\ '  /\    \ /_/                / /  ___
 *          |    == o ==      |       /|         / /  / _ \
 *           \      '        /       | |        / /__|  __/
 *             \           /         \ \        \____/\___|
 *             /----<o>---- \         / /        __  __  __  __      __        ___
 *             |            ' \       \ \       |__)/  \|__)/  \ __ /  |__| /\  |
 *             |    |    | '   '\      \ \      | \ \__/|__)\__/    \__|  |/--\ |
 *  _________  | ´´ |  ' |     '  \    / /
 *  |  MAYA  | |  ' |    | '       |__/ /
 *   \______/   \__/ \__/ \_______/____/
 *
 * @file uros_motor_app.c
 * @author Alejandro Gomez Molina (@Alejo2313)
 * @brief uROS motor control application.
 *
 * @version 0.1
 * @date Feb 20, 2023
 *
 * @copyright Copyright (c) 2023
 *
 */

/************************************************************************
    INCLUDES
************************************************************************/
#include "uros_motor_app.h"

/************************************************************************
    PRIVATE DEFINES AND TYPES
************************************************************************/

typedef geometry_msgs__msg__TwistStamped TwistStamped_msg_t;
typedef geometry_msgs__msg__PoseStamped PoseStamped_msg_t;
typedef nav_msgs__msg__Odometry Odometry_msg_t;

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

static TwistStamped_msg_t UROS_MOTOR_newSpeed;
static TwistStamped_msg_t UROS_MOTOR_currentSpeed;
static PoseStamped_msg_t UROS_MOTOR_newRotation;
static PoseStamped_msg_t UROS_MOTOR_currentRotation;

static rcl_subscription_t UROS_MOTOR_speedSub;
static rcl_subscription_t UROS_MOTOR_rotationSub;
static rcl_publisher_t UROS_MOTOR_speedPub;
static rcl_publisher_t UROS_MOTOR_rotationPub;
static rcl_timer_t UROS_MOTOR_pubTimer;

/************************************************************************
    FUNCTIONS
************************************************************************/

/**
 * @brief Speed subscription callback. Called on new data.
 * @param data Pointer to incoming message.
 */
static void UROS_MOTOR_speedCallback (const void* data) {
  const TwistStamped_msg_t* speedData = (TwistStamped_msg_t*)data;
  float speed_x = speedData->twist.linear.x;
  float angle_x = speedData->twist.angular.x;
  /* ToDo: Convert linear to angular position */

  /* Update controller */
  NOMAD_WHEEL_setPoint(speed_x, angle_x);
}

/**
 * @brief Rotation subscription callback. Called on new data.
 * @param data Pointer to incoming message.
 */
static void UROS_MOTOR_rotationCallback (const void* data) {
  const PoseStamped_msg_t* rotationData = (PoseStamped_msg_t*)data;
  /* ToDo: Cast to correct type */
  NOMAD_ROTATION_setPoint(rotationData->pose.orientation.x);
}


/**
 * @brief Periodic publisher callback.
 * @param timer uROS timer instance.
 * @param last_call_time Last call time.
 */
static void UROS_MOTOR_timerCallback (rcl_timer_t * timer, int64_t last_call_time) {
  rcl_ret_t result;

  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(result);
  memset(&UROS_MOTOR_currentSpeed, 0, sizeof(TwistStamped_msg_t));
  memset(&UROS_MOTOR_currentRotation, 0, sizeof(PoseStamped_msg_t));

  if (timer != NULL) {
    /* Get the data */
    UROS_MOTOR_currentSpeed.twist.angular.x = NOMAD_WHEEL_getRotation();
    UROS_MOTOR_currentSpeed.twist.linear.x = NOMAD_WHEEL_getSpeed();
    UROS_MOTOR_currentRotation.pose.orientation.x = NOMAD_ROTATION_getRotation();
    /* Publish data */
    result = rcl_publish(&UROS_MOTOR_speedPub, &UROS_MOTOR_currentSpeed, NULL);
    result = rcl_publish(&UROS_MOTOR_rotationPub, &UROS_MOTOR_currentRotation, NULL);
  }

}

/**
 * @brief Setup memory allocators and tasks.
 */
static void UROS_MOTOR_setup () {

  NOMAD_WHEEL_Init();
  NOMAD_ROTATION_Init();

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
    while(1);
  }
}

/**
 * @brief Motor application main function.
 */
static void UROS_MOTOR_TaskFn () {
  rcl_node_t node;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
  rclc_support_t support;

  rcl_ret_t result;
  RCL_UNUSED(result);

  UROS_MOTOR_setup();
  allocator = rcl_get_default_allocator();
  result = rclc_support_init(&support, 0, NULL, &allocator);
  result = rclc_node_init_default(&node, UROS_MOTOR_NODE_NAME, UROS_MOTOR_NODE_NS, &support);

  result = rclc_subscription_init_default(
      &UROS_MOTOR_speedSub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
      UROS_MOTOR_SUB_SPEED);

  result = rclc_subscription_init_default(
      &UROS_MOTOR_rotationSub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
      UROS_MOTOR_SUB_ROTATION);

  result = rclc_publisher_init_default(
      &UROS_MOTOR_speedPub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
      UROS_MOTOR_PUB_SPEED);

  result = rclc_publisher_init_default(
      &UROS_MOTOR_rotationPub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
      UROS_MOTOR_PUB_SPEED);

  result = rclc_timer_init_default(
      &UROS_MOTOR_pubTimer,
      &support,
      RCL_MS_TO_NS(UROS_MOTOR_PUB_PERIOD_MS),
      UROS_MOTOR_timerCallback);


  result = rclc_executor_init(
      &executor,
      &support.context,
      UROS_MOTOR_N_INSTANCES,
      &allocator);


  result = rclc_executor_add_subscription(
      &executor,
      &UROS_MOTOR_speedSub,
      &UROS_MOTOR_newSpeed,
      UROS_MOTOR_speedCallback,
      ON_NEW_DATA);

  result = rclc_executor_add_subscription(
      &executor,
      &UROS_MOTOR_rotationSub,
      &UROS_MOTOR_newRotation,
      UROS_MOTOR_rotationCallback,
      ON_NEW_DATA);

  result = rclc_executor_add_timer(&executor, &UROS_MOTOR_pubTimer);

  RCL_UNUSED(result);

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(UROS_MOTOR_SPIN_TIME_MS));
    vTaskDelay(pdMS_TO_TICKS(UROS_MOTOR_DELAY_TIME_MS));
  }
}

/**
 * @brief Initalize microROS task
 */
void UROS_MOTOR_init () {
  /* ToDo: Check if the task has been created correctly */
  xTaskCreate(
      UROS_MOTOR_TaskFn,
      UROS_MOTOR_TASK_NAME,
      UROS_MOTOR_TASK_STACK,
      NULL,
      UROS_MOTOR_TASK_PRIO,
      NULL);
}
