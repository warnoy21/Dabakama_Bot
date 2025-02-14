#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <example_interfaces/msg/float32.h>
#include "pwm.h"
rcl_subscription_t speed_subscriber;
example_interfaces__msg__Float32 speed_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void wheel_speed_callback(const void * msgin)
{  


 const example_interfaces__msg__Float32 *floatmsg = (const example_interfaces__msg__Float32 *)msgin;

  PWM_A.period(0.01f); // 100Hz
  PWM_B.period(0.01f);
  pwm_val = floatmsg->data;
  max_pwm_wheels(); // Scale PWM based on speed_msg

  pwm_valA = constrain(pwm_valA, 0.0, max_pwm);
  pwm_valB = constrain(pwm_valB, 0.0, max_pwm);

  PWM_A.write(pwm_valA);
  PWM_B.write(pwm_valB);

  Serial.print("PWM A: ");
  Serial.print(pwm_valA);
  Serial.print(", PWM B: ");
  Serial.println(pwm_valB);

}

void setup() {
   set_microros_wifi_transports(
    (char*)"COGECO-4BFA0",
    (char*)"CA299COGECO",
    (char*)"192.168.0.21",
    8888
  );
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &speed_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(example_interfaces, msg, Float32),
    "wheel_speed_value"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &speed_subscriber, &speed_msg, &wheel_speed_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
