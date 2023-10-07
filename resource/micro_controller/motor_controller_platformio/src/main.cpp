#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>

#include <std_msgs/msg/int32.h>
#include <ESP32Encoder.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

const int r1=26,r2=27,l1=12,l2=14,speedr=25,speedl=13;

rcl_subscription_t subscriber;
rcl_publisher_t publisher_left;
rcl_publisher_t publisher_right;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Int32 msg_publisher_left;
std_msgs__msg__Int32 msg_publisher_right;
rclc_executor_t executor;
rclc_executor_t executor_publisher;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

int speed_factor = 50;
const int turn_interval = 50; // Adjust this value to control the turn interval (in milliseconds)

float global_left_speed = 0;
float global_right_speed = 0;
float global_interval = 0;
unsigned long previousMillis = -5000; 

ESP32Encoder encoder_left;
ESP32Encoder encoder_right;

void error_loop(){
  while(1){
    delay(100);
  }
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    msg_publisher_left.data = (int32_t)encoder_left.getCount();
    msg_publisher_right.data = (int32_t)encoder_right.getCount();
    RCSOFTCHECK(rcl_publish(&publisher_left, &msg_publisher_left, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_right, &msg_publisher_right, NULL));
    
  }
}

void move_rover(float lspeed, float rspeed) {

  if (lspeed > 0) {
    digitalWrite(l1,LOW);
    digitalWrite(l2,HIGH);
    analogWrite(speedl, lspeed); 
  }
  else if (lspeed < 0) {
    digitalWrite(l1,HIGH);
    digitalWrite(l2,LOW);
    analogWrite(speedl, -lspeed);     
  }
  else {
    digitalWrite(l1,LOW);
    digitalWrite(l2,LOW);
    analogWrite(speedl, 0);    
  }

  if (rspeed > 0) {
    digitalWrite(r1,HIGH);
    digitalWrite(r2,LOW);
    analogWrite(speedr, rspeed); 
  }
  else if (rspeed < 0) {
    digitalWrite(r1,LOW);
    digitalWrite(r2,HIGH);
    analogWrite(speedr, -rspeed);     
  }
  else {
    digitalWrite(r1,LOW);
    digitalWrite(r2,LOW);
    analogWrite(speedr, 0);    
  }
  
}

//twist message cb
void subscription_callback_simple(const void *msgin) {
  // Cast the received message to the appropriate type
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Extract linear and angular velocities from the message
  float linear_velocity = msg->linear.x;
  float angular_velocity = msg->angular.z;
  
  // Initialize left and right wheel speeds
  float left_speed = 0;
  float right_speed = 0;
  float interval = 0;

  // Check the type of movement based on the angular velocity
  if (angular_velocity > 0.1) {
    // Robot is turning right
    left_speed = -200;  
    right_speed = 200;
    interval = angular_velocity * 100 + 150;         
  } else if (angular_velocity < -0.1) {
    // Robot is turning left
    left_speed = 200;
    right_speed = -200;               
    interval = -angular_velocity * 100 + 150;
  } else if (linear_velocity > 0.1) {
    // Robot is moving forward
    left_speed = 108;  
    right_speed = 100;
  } else if (linear_velocity < -0.1) {
    // Robot is moving backward
    left_speed = -108;  
    right_speed = -100;   
  } 

  global_left_speed = left_speed;
  global_right_speed = right_speed;
  global_interval = interval;

}



void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  //ESP32Encoder::useInternalWeakPullResistors=DOWN;
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;

  // use pin 19 and 18 for the first encoder
  encoder_left.attachHalfQuad(35, 34);
  // use pin 17 and 16 for the second encoder
  encoder_right.attachHalfQuad(33, 32);

  pinMode(l1,OUTPUT);   //left motors forward
  pinMode(l2,OUTPUT);
  pinMode(r1,OUTPUT);   //right motors forward
  pinMode(r2,OUTPUT);   //right motors reverse
  pinMode(speedl,OUTPUT);
  pinMode(speedr,OUTPUT);
  
  delay(1000);

  state = WAITING_AGENT;
  
  delay(100);

 }

bool create_entities() { 
  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "lwheel"));


  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "rwheel"));


  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback_simple, ON_NEW_DATA));
  
  // create executor_publisher
  RCCHECK(rclc_executor_init(&executor_publisher, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_publisher, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  //(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_left,  &node);
  rcl_publisher_fini(&publisher_right, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_executor_fini(&executor_publisher);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void loop() {

 // delay(100); 
 switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
      if (state == AGENT_CONNECTED) {
        if (millis() - previousMillis > 200 ) {
            move_rover(global_left_speed, global_right_speed);
        
            if (global_interval > 0 ){ 
              previousMillis = millis();
              delay(global_interval);
              move_rover(0, 0);
            }
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_publisher, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }  

}

