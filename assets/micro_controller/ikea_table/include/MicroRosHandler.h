#ifndef MICROROS_HANDLER_H
#define MICROROS_HANDLER_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

#include "MotorControl.h"
#include "EncoderController.h"

class MicroRosHandler {
public:
  MicroRosHandler(MotorControl &motorControl, EncoderController &encoderController);
  bool init();
  void update();
  void destroy();

private:
  static MicroRosHandler* instance;

  // Referenzen auf Controller
  MotorControl &motorCtrl;
  EncoderController &encoders;

  // ROS Entities
  rcl_node_t node;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
  rclc_support_t support;

  // Publisher
  rcl_publisher_t pub_left_ticks;
  rcl_publisher_t pub_right_ticks;

  // Subscriber
  rcl_subscription_t sub_motor;

  // Timer
  rcl_timer_t encoder_timer;

  // Nachrichtenobjekte
  geometry_msgs__msg__Twist msg_motor;
  std_msgs__msg__Int32 msg_left_ticks;
  std_msgs__msg__Int32 msg_right_ticks;

  // Zugriff für externen Executor
  geometry_msgs__msg__Twist* getMsgMotor() { return &msg_motor; }

  // Statische Callback-Funktionen
  static void motorCallback(const void *msgin);
  static void encoder_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

};

#endif // MICROROS_HANDLER_H
