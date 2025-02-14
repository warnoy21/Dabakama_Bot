/*
 * Description: Microros for Portenta H7 control the motors via a subscriber. The node is "micro_ros_arduino_node"
 * and subscribes to "motor_value_topic" for motor inputs.
 *              
 * 
 * Version 1.0      Date: Janurary 21,2025 Initial Creation                 Made by: Aaron Gumba
 * Version 1.0.1    Date  February 14,2025 Integrated HC_SR04 sensor                 Aaron Gumba
 *



 */
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <example_interfaces/msg/float32.h>
#include <std_msgs/msg/string.h>
#include "hc_sr04_publisher.h"
#include "wheel_controller.h"

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

rcl_subscription_t speed_subscriber;
example_interfaces__msg__Float32 speed_msg;
rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t obstacle_distance_publisher;
std_msgs__msg__String distance_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
// Callback function for publishing distance data
void obstacle_distance_callback(rcl_timer_t *timer, int64_t last_call_time) {  
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Get the distance as a String
        String distance_str = ultraSonicDistance();

        // Allocate memory for distance message
        size_t size = distance_str.length() + 1; // Including null terminator
        if (distance_msg.data.data != NULL) {
            free(distance_msg.data.data); // Free previous allocation
        }
        distance_msg.data.data = (char*)malloc(size);
        
        if (distance_msg.data.data != NULL) {
            strcpy(distance_msg.data.data, distance_str.c_str());
            distance_msg.data.size = size - 1; // Exclude null terminator
            distance_msg.data.capacity = size;

            // Publish the message
            RCSOFTCHECK(rcl_publish(&obstacle_distance_publisher, &distance_msg, NULL));
        }
    }
}
void wheel_speed_callback(const void * msgin)
{  
 const example_interfaces__msg__Float32 *floatmsg = (const example_interfaces__msg__Float32 *)msgin;
 pwm_val = floatmsg->data;
 max_pwm = 1.0 * pwm_val;  // ? Update max_pwm dynamically
 
}

void motor_subscription_callback(const void *msgin) {
  
  PWM_A.period(0.01f); //100HZ
  PWM_B.period(0.01f);

  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  //(-0.051 <=x<= 0.051)   threshold for noise stop motor off
  if (msg->linear.x <= -0.051){ 
    forward_wheels(); 
  }

  else if (msg->linear.x > 0.051){
    backward_wheels();
  }

  else if (msg->angular.z >0.99){
    full_right_turn();
  }

  else if (msg->angular.z <-0.99){
    full_left_turn();
  }
    
  else{
    stop_wheels();
  }

  //-0.2 <= x <= 0.2 threshold to run pwm max due to noise or human use of analogstick inprecise straight
  if ((msg->angular.z <= 0.2) && (msg->angular.z >= -0.2)){
    max_pwm_wheels();
  }
    
   
  else if (msg->angular.z > 0){
    right_angle(msg);
  }

  else {
    left_angle(msg);
  }
  pwm_valA = constrain(pwm_valA, 0.0, max_pwm); // Clamp PWM to valid range
  pwm_valB = constrain(pwm_valB, 0.0, max_pwm); // Clamp PWM to valid range

  PWM_A.write(pwm_valA);
  PWM_B.write(pwm_valB);

  Serial.print("PWM A: ");
  Serial.print(pwm_valA);
  Serial.print(", PWM B: ");
  Serial.println(pwm_valB);

}


void setup() {
  //Serial.begin(115200);
 // while (!Serial); // Wait for serial to initialize

 pinMode(LED_PIN, OUTPUT);
  // Initialize the pin to LOW (Motors off)
  MOTOR_A1 = 0;
  MOTOR_A2 = 0;
  MOTOR_B1 = 0;
  MOTOR_B2 = 0;

  set_microros_wifi_transports(
    (char*)"COGECO-4BFA0",
    (char*)"CA299COGECO",
    (char*)"192.168.0.21",
    8888
  );

  


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscribers
  RCCHECK(rclc_subscription_init_default(
    &speed_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(example_interfaces, msg, Float32),
    "wheel_speed_value"));
  RCCHECK(rclc_subscription_init_default(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "motor_value_topic"
  ));

  // Create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &obstacle_distance_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "obstackle_avoidance_publisher"));
  // Create timer (trigger every 500ms)
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(500),
    obstacle_distance_callback));

  

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &speed_subscriber, 
    &speed_msg, 
    &wheel_speed_callback, 
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &motor_subscriber,
    &msg,
    &motor_subscription_callback,
    ON_NEW_DATA
  ));
   RCCHECK(rclc_executor_add_timer(&executor, &timer));
 
  
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(100);
}
