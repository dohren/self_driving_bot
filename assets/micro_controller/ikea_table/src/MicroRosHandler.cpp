#include "MicroRosHandler.h"
#include <Arduino.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>

// Statische Instanzinitialisierung
MicroRosHandler* MicroRosHandler::instance = nullptr;

// Konstruktor
MicroRosHandler::MicroRosHandler(
  MotorControl &motorControl,
  EncoderController &encoderController
) : motorCtrl(motorControl), encoders(encoderController)
{
  instance = this; // für statische Callbacks
}

// Initialisierung von ROS-Node, Publishern, Subscribern, Timern
bool MicroRosHandler::init() {
  allocator = rcl_get_default_allocator();
  Serial.println("[MicroRosHandler] initialisierung.");
  

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
    Serial.println("[MicroRosHandler] Support init failed.");
    return false;
  }

  if (rclc_node_init_default(&node, "micro_ros_handler_node", "", &support) != RCL_RET_OK) {
    Serial.println("[MicroRosHandler] Node init failed.");
    return false;
  }

  // Subscriptions
  rclc_subscription_init_default(
    &sub_motor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_transform"
  );

  // Publishers
  rclc_publisher_init_default(
    &pub_left_ticks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_ticks"
  );

  rclc_publisher_init_default(
    &pub_right_ticks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_ticks"
  );
 
  // Timer für Encoder
  rclc_timer_init_default(
    &encoder_timer,
    &support,
    RCL_MS_TO_NS(100),
    encoder_timer_callback
  );


  // Executor
  rclc_executor_init(&executor, &support.context, 9, &allocator);

  rclc_executor_add_subscription(&executor, &sub_motor, &msg_motor, &motorCallback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &encoder_timer);

  Serial.println("[MicroRosHandler] Initialized successfully.");
  return true;
}

// Regelmäßiger Aufruf in loop()
void MicroRosHandler::update() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

// Ressourcen freigeben
void MicroRosHandler::destroy() {
  //rcl_publisher_fini(&pub_lidar, &node);
  rcl_subscription_fini(&sub_motor, &node);
  rcl_timer_fini(&encoder_timer);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
  Serial.println("[MicroRosHandler] Destroyed.");
}

// ==============================
//       Callback-Funktionen
// ==============================

void MicroRosHandler::motorCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = static_cast<const geometry_msgs__msg__Twist *>(msgin);

  float linear = msg->linear.x;
  float angular = -msg->angular.z;

  float base_speed = 255.0f;   // Vorwärtsgeschwindigkeit
  float turn_speed = 155.0f;   // Drehkomponente

  float left = linear * base_speed - angular * turn_speed;
  float right = linear * base_speed + angular * turn_speed;

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  Serial.printf("[MicroRosHandler] motorCallback: linear=%.2f angular=%.2f → L=%.1f R=%.1f\n",
                linear, angular, left, right);

  instance->motorCtrl.setSpeed(left, right);
}

void MicroRosHandler::encoder_timer_callback(rcl_timer_t *timer, int64_t) {
  if (timer != nullptr) {
    instance->msg_left_ticks.data = instance->encoders.getLeftCount();
    instance->msg_right_ticks.data = instance->encoders.getRightCount();

    rcl_publish(&instance->pub_left_ticks, &instance->msg_left_ticks, nullptr);
    rcl_publish(&instance->pub_right_ticks, &instance->msg_right_ticks, nullptr);
  }
}


