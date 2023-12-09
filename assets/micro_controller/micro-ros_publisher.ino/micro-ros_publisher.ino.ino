#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>
#include "LD06forArduino.h"
#include <algorithm>


rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

LD06forArduino ld06;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rmw_uros_sync_session(200));  
    msg.header.stamp.sec = int(rmw_uros_epoch_millis() / 1000);
    msg.header.stamp.nanosec = rmw_uros_epoch_nanos() % 1000000000; ;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  set_microros_transports();
  ld06.Init(13);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "lidar_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"));
    
  // create timer,
  const unsigned int timer_timeout = 200;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  ld06.read_lidar_data();
  msg.ranges.capacity = 360;
  msg.ranges.size =  360;
  msg.ranges.data =  (float*)malloc(360 * sizeof(float));

  msg.intensities.capacity = 360;
  msg.intensities.size =  360;
  msg.intensities.data =  (float*)malloc(360 * sizeof(float));

  msg.header.frame_id.data = "base_laser"; 
  msg.angle_min = 0.0;
  msg.angle_max = 6.28000020980835;
  msg.angle_increment = 0.01749303564429283;
  msg.time_increment = 0.0;
  msg.scan_time = 0.0;
  msg.range_min = 0.05;
  msg.range_max = 8.0;
  
}

void loop() {
  ld06.read_lidar_data();
  for (int i = 0; i < ld06.data_length; i++) {
    msg.ranges.data[int(359 - ld06.angles[i])] = ld06.distances[i]/1000;
    msg.intensities.data[359 - int(ld06.angles[i])] = ld06.confidences[i];
  }
  
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

 
