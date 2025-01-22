/*
 * Description: Microros for Portenta H7 control the motors via a subscriber. The node is "micro_ros_arduino_node"
 * and subscribes to "motor_value_topic" for motor inputs.
 *              
 * 
 * Version 1.0      Date: Janurary 21,2025 Initial Creation         Made by: Aaron Gumba
 * 
 */








#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include "mbed.h" 

#define LED_PIN 13

#define PWM_pinA PH_6   //PWM_9
//#define PWM_pinB PJ_10  //PWM_8

mbed::PwmOut PWM_A(PWM_pinA);  // Create a PWM object
//mbed::PwmOut PWM_B(PWM_pinB);
//
#define MOTOR_pin_A1 PC_13  // GPIO 0
#define MOTOR_pin_A2 PC_15  // GPIO 1
//#define MOTOR_pin_B1 PD_4   // GPIO 2
//#define MOTOR_pin_B2 PD_5   // GPIO 3
//
mbed::DigitalOut MOTOR_A1(MOTOR_pin_A1); 
mbed::DigitalOut MOTOR_A2(MOTOR_pin_A2); 
//mbed::DigitalOut MOTOR_B1(MOTOR_pin_B1); 
//mbed::DigitalOut MOTOR_B2(MOTOR_pin_B2); 

float max_pwm = 1.0;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

rcl_subscription_t motor_subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void motor_subscription_callback(const void *msgin) {
  PWM_A.period(0.01f);
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  //(-0.09 <=x<= 0.09)   threshold for noise stop motor off
  if (msg->linear.x <= -0.09){ //threshold for noise stop motor off

   MOTOR_A1 = 1;
   MOTOR_A2 = 0;
    }

   else if (msg->linear.x > 0.09){

    MOTOR_A1 = 0;
    MOTOR_A2 = 1;
    
    }

   else{

    MOTOR_A1 = 0;
    MOTOR_A2 = 0;
     
    }
  float pwm_val;
  //-0.2 <= x <= 0.2 threshold to run pwm max due to noise or human use of analogstick inprecise straight
   if ((msg->angular.z <= 0.2) && (msg->angular.z >= -0.2)){
    
    pwm_val = max_pwm;
    
    }
    
   
   else if (msg->angular.z > 0){
      pwm_val = max_pwm - msg->angular.z ;
    }

    else {
      pwm_val = fabs(msg->angular.z) ;
      }
    pwm_val = constrain(pwm_val, 0.0, max_pwm); // Clamp PWM to valid range
    PWM_A.write(pwm_val);

    
}

void setup() {


 pinMode(LED_PIN, OUTPUT);
  // Initialize the pin to LOW (Motors off)
  MOTOR_A1 = 0;
  MOTOR_A2 = 0;
//  MOTOR_B1 = 0;
//  MOTOR_B2 = 0; 
//
//  PWM_B.period(0.02f);  // 20 kHz period (50 microseconds)
//  PWM_B.write(0.0f);    // Start off (0% duty cycle)
//
 







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

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "motor_value_topic"
  ));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &motor_subscriber,
    &msg,
    &motor_subscription_callback,
    ON_NEW_DATA
  ));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(100);
}
