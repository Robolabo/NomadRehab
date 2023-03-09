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


#if defined(UROS_MOTOR_STAMPED_DATA) && (UROS_MOTOR_STAMPED_DATA == 1)
typedef geometry_msgs__msg__TwistStamped Twist_msg_t;
typedef geometry_msgs__msg__PoseStamped Pose_msg_t;

#define UROS_MOTOR_TWIST_TYPE_SUPPORT  (ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped))
#define UROS_MOTOR_POSE_TYPE_SUPPORT  (ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped))

#else
typedef geometry_msgs__msg__Twist Twist_msg_t;
typedef geometry_msgs__msg__Pose Pose_msg_t;
#define UROS_MOTOR_TWIST_TYPE_SUPPORT  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist)
#define UROS_MOTOR_POSE_TYPE_SUPPORT  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose)
#endif

typedef nav_msgs__msg__Odometry Odometry_msg_t;

/************************************************************************
    PRIVATE DECLARATIONS
************************************************************************/

static Twist_msg_t UROS_MOTOR_newSpeed;
static Twist_msg_t UROS_MOTOR_currentSpeed;

static Odometry_msg_t UROS_MOTOR_wheel_odometry;

static Pose_msg_t UROS_MOTOR_newRotation;
static Pose_msg_t UROS_MOTOR_currentRotation;

static rcl_subscription_t UROS_MOTOR_speedSub;
static rcl_subscription_t UROS_MOTOR_rotationSub;
static rcl_publisher_t UROS_MOTOR_speedPub;
static rcl_publisher_t UROS_MOTOR_rotationPub;
static rcl_timer_t UROS_MOTOR_pubTimer;


static rcl_node_t UROS_MOTOR_node;
static rclc_executor_t UROS_MOTOR_executor;
static rcl_allocator_t UROS_MOTOR_allocator;
static rclc_support_t UROS_MOTOR_support;

static bool UROS_MOTOR_restart = false;
static uint16_t UROS_MOTOR_errors = 0;

/************************************************************************
    FUNCTIONS
************************************************************************/
#define UROS_MOTOR_MAX_ERROS  5U
#define UROS_MOTOR_CHECK(ret) { \
  if ((ret) != RMW_RET_OK ) {   \
    UROS_MOTOR_errors++;        \
    }                           \
   if(UROS_MOTOR_errors > (UROS_MOTOR_MAX_ERROS)) {  \
     UROS_MOTOR_restart = true; \
   }                            \
}



/**
 * @brief Speed subscription callback. Called on new data.
 * @param data Pointer to incoming message.
 */
static void UROS_MOTOR_speedCallback (const void* data) {
  const Twist_msg_t* speedData = (Twist_msg_t*)data;
#if defined(UROS_MOTOR_STAMPED_DATA) && (UROS_MOTOR_STAMPED_DATA == 1)
  float speed_x = speedData->twist.linear.x;
  float angle_x = speedData->twist.angular.x;
#else
  float velocity = sqrt(pow(speedData->linear.x, 2) + pow(speedData->linear.y, 2));
  float theta = atan2(speedData->linear.y, speedData->linear.x);
#endif
  /* ToDo: Convert linear to angular position */

  /* Update controller */
  NOMAD_WHEEL_setPoint(velocity, theta);
}

/**
 * @brief Rotation subscription callback. Called on new data.
 * @param data Pointer to incoming message.
 */
static void UROS_MOTOR_rotationCallback (const void* data) {
  const Pose_msg_t* rotationData = (Pose_msg_t*)data;
  /* ToDo: Cast to correct type */
#if defined(UROS_MOTOR_STAMPED_DATA) && (UROS_MOTOR_STAMPED_DATA == 1)
  NOMAD_ROTATION_setPoint(rotationData->pose.orientation.x);
#else
  NOMAD_ROTATION_setPoint(rotationData->orientation.x);
#endif
}


/**
 * @brief Periodic publisher callback.
 * @param timer uROS timer instance.
 * @param last_call_time Last call time.
 */
static void UROS_MOTOR_timerCallback (rcl_timer_t * timer, int64_t last_call_time) {
  static uint32_t sec = 0;
  rcl_ret_t result;

  NOMAD_WHEEL_Odometry_t odometry;

  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(result);
  memset(&UROS_MOTOR_currentSpeed, 0, sizeof(Twist_msg_t));
  memset(&UROS_MOTOR_currentRotation, 0, sizeof(Pose_msg_t));
  memset(&UROS_MOTOR_wheel_odometry, 0, sizeof(Odometry_msg_t));

  if (timer != NULL) {
    /* Get the data */
#if defined(UROS_MOTOR_STAMPED_DATA) && (UROS_MOTOR_STAMPED_DATA == 1)
    UROS_MOTOR_currentSpeed.header.stamp.sec = sec;
    UROS_MOTOR_currentRotation.header.stamp.sec = sec;
    UROS_MOTOR_currentSpeed.header.stamp.nanosec = xTaskGetTickCount();
    UROS_MOTOR_currentRotation.header.stamp.nanosec = xTaskGetTickCount();

    UROS_MOTOR_currentSpeed.twist.angular.z = NOMAD_WHEEL_getRotation();
    UROS_MOTOR_currentSpeed.twist.linear.x = NOMAD_WHEEL_getSpeed();
    UROS_MOTOR_currentRotation.pose.orientation.z = NOMAD_ROTATION_getRotation();
#else
    UROS_MOTOR_currentSpeed.angular.z = NOMAD_WHEEL_getRotation();
    UROS_MOTOR_currentSpeed.linear.x = NOMAD_WHEEL_getSpeed();
    UROS_MOTOR_currentRotation.orientation.z = NOMAD_ROTATION_getRotation();
#endif

    NOMAD_WHEEL_getOdometry(&odometry);

    UROS_MOTOR_wheel_odometry.pose.pose.position.x = odometry.x;
    UROS_MOTOR_wheel_odometry.pose.pose.position.y = odometry.y;
    UROS_MOTOR_wheel_odometry.twist.twist.linear.x = odometry.v_x;
    UROS_MOTOR_wheel_odometry.twist.twist.linear.x = odometry.v_y;
    UROS_MOTOR_wheel_odometry.twist.twist.angular.z = odometry.v_th;

    /* Publish data */
    UROS_MOTOR_CHECK(rcl_publish(&UROS_MOTOR_speedPub, &UROS_MOTOR_currentSpeed, NULL));
    UROS_MOTOR_CHECK(rcl_publish(&UROS_MOTOR_rotationPub, &UROS_MOTOR_currentRotation, NULL));
    sec++;

  }

}

/**
 * @brief Setup memory allocators and tasks.
 */
static void UROS_MOTOR_setup () {

  NOMAD_WHEEL_Init();
  NOMAD_ROTATION_Init();

  NOMAD_WHEEL_enableControl();


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
static void UROS_MOTOR_appInit () {
  rcl_ret_t result;
  RCL_UNUSED(result);
  const char* node_name = UROS_MOTOR_NODE_NAME;
  UROS_MOTOR_allocator = rcl_get_default_allocator();
  result = rclc_support_init(&UROS_MOTOR_support, 0, NULL, &UROS_MOTOR_allocator);
  result = rclc_node_init_default(&UROS_MOTOR_node, node_name, UROS_MOTOR_NODE_NS, &UROS_MOTOR_support);

  result = rclc_subscription_init_best_effort(
      &UROS_MOTOR_speedSub,
      &UROS_MOTOR_node,
      UROS_MOTOR_TWIST_TYPE_SUPPORT,
      UROS_MOTOR_SUB_SPEED);

  result = rclc_subscription_init_best_effort(
      &UROS_MOTOR_rotationSub,
      &UROS_MOTOR_node,
      UROS_MOTOR_POSE_TYPE_SUPPORT,
      UROS_MOTOR_SUB_ROTATION);

  result = rclc_publisher_init_default(
      &UROS_MOTOR_speedPub,
      &UROS_MOTOR_node,
      UROS_MOTOR_TWIST_TYPE_SUPPORT,
      UROS_MOTOR_PUB_SPEED);

  result = rclc_publisher_init_default(
      &UROS_MOTOR_rotationPub,
      &UROS_MOTOR_node,
      UROS_MOTOR_POSE_TYPE_SUPPORT,
      UROS_MOTOR_PUB_ROTATION);

  result = rclc_timer_init_default(
      &UROS_MOTOR_pubTimer,
      &UROS_MOTOR_support,
      RCL_MS_TO_NS(UROS_MOTOR_PUB_PERIOD_MS),
      UROS_MOTOR_timerCallback);

  result = rclc_executor_init(
      &UROS_MOTOR_executor,
      &UROS_MOTOR_support.context,
      UROS_MOTOR_N_INSTANCES,
      &UROS_MOTOR_allocator);

  result = rclc_executor_add_subscription(
      &UROS_MOTOR_executor,
      &UROS_MOTOR_speedSub,
      &UROS_MOTOR_newSpeed,
      UROS_MOTOR_speedCallback,
      ON_NEW_DATA);

  result = rclc_executor_add_subscription(
      &UROS_MOTOR_executor,
      &UROS_MOTOR_rotationSub,
      &UROS_MOTOR_newRotation,
      UROS_MOTOR_rotationCallback,
      ON_NEW_DATA);

  result = rclc_executor_add_timer(&UROS_MOTOR_executor, &UROS_MOTOR_pubTimer);
}


static void UROS_MOTOR_appFinit () {
  NOMAD_WHEEL_saveContext();
  NOMAD_ROTATION_saveContext();
  NOMAD_ROTATION_disableControl();
  NOMAD_WHEEL_disableControl();
  NVIC_SystemReset();
}
/**
 * @brief Motor application main function.
 */
static void UROS_MOTOR_TaskFn () {

  while (1) {
    UROS_MOTOR_appInit();
    UROS_MOTOR_restart = false;
    while (!UROS_MOTOR_restart) {
      UROS_MOTOR_CHECK(rclc_executor_spin_some(&UROS_MOTOR_executor, RCL_MS_TO_NS(UROS_MOTOR_SPIN_TIME_MS)));
      vTaskDelay(pdMS_TO_TICKS(UROS_MOTOR_DELAY_TIME_MS));
    }
    UROS_MOTOR_appFinit();
  }
}

/**
 * @brief Initalize microROS task
 */
void UROS_MOTOR_init () {
  /* ToDo: Check if the task has been created correctly */
  UROS_MOTOR_setup();

  xTaskCreate(
      UROS_MOTOR_TaskFn,
      UROS_MOTOR_TASK_NAME,
      UROS_MOTOR_TASK_STACK,
      NULL,
      UROS_MOTOR_TASK_PRIO,
      NULL);
}
