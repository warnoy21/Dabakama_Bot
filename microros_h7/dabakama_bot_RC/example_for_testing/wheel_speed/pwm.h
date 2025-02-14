#include "mbed.h" 
#include <micro_ros_arduino.h>

#include <example_interfaces/msg/float32.h>

#define PWM_pinA PH_6   //PWM_9
#define PWM_pinB PJ_10  //PWM_8



mbed::PwmOut PWM_A(PWM_pinA);  // Create a PWM object
mbed::PwmOut PWM_B(PWM_pinB);

float pwm_val;
float max_pwm = 1.0 * pwm_val;
float pwm_valA;
float pwm_valB;

void max_pwm_wheels(void){
  pwm_valA = max_pwm;
  pwm_valB = max_pwm;
}

void right_angle(const geometry_msgs__msg__Twist *msg){
  //right turn
  pwm_valA = msg->angular.z * pwm_val ;
  pwm_valB = max_pwm ;
}

void left_angle(const geometry_msgs__msg__Twist *msg, example_interfaces__msg__Float32 * floatmsg){
  //left turn
  pwm_valA = max_pwm ;
  pwm_valB = (fabs(msg->angular.z)) * pwm_val ;
}