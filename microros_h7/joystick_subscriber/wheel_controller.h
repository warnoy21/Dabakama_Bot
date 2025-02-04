#include "mbed.h" 
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>

#define PWM_pinA PH_6   //PWM_9
#define PWM_pinB PJ_10  //PWM_8



mbed::PwmOut PWM_A(PWM_pinA);  // Create a PWM object
mbed::PwmOut PWM_B(PWM_pinB);


#define MOTOR_pin_A1 PC_13  // GPIO 0
#define MOTOR_pin_A2 PC_15  // GPIO 1
#define MOTOR_pin_B1 PD_4   // GPIO 2
#define MOTOR_pin_B2 PD_5   // GPIO 3

mbed::DigitalOut MOTOR_A1(MOTOR_pin_A1); 
mbed::DigitalOut MOTOR_A2(MOTOR_pin_A2); 
mbed::DigitalOut MOTOR_B1(MOTOR_pin_B1); 
mbed::DigitalOut MOTOR_B2(MOTOR_pin_B2); 

float max_pwm = 1.0;
float pwm_valA;
float pwm_valB;

void forward_wheels(void){
  MOTOR_A1 = 1;
  MOTOR_A2 = 0;
  MOTOR_B1 = 1;
  MOTOR_B2 = 0;

}

void backward_wheels(void){
  MOTOR_A1 = 0;
  MOTOR_A2 = 1;
  MOTOR_B1 = 0;
  MOTOR_B2 = 1; 
}

void full_right_turn(void){
  MOTOR_A1 = 1;
  MOTOR_A2 = 0;
  MOTOR_B1 = 0;
  MOTOR_B2 = 1;
}


void full_left_turn(void){
  MOTOR_A1 = 0;
  MOTOR_A2 = 1;
  MOTOR_B1 = 1;
  MOTOR_B2 = 0;
}
void stop_wheels(void){
  MOTOR_A1 = 0;
  MOTOR_A2 = 0;
  MOTOR_B1 = 0;
  MOTOR_B2 = 0;
}

void max_pwm_wheels(void){
  pwm_valA = max_pwm;
  pwm_valB = max_pwm;
}

void right_angle(const geometry_msgs__msg__Twist *msg){
  //right turn
  pwm_valA = msg->angular.z ;
  pwm_valB = max_pwm;
}

void left_angle(const geometry_msgs__msg__Twist *msg){
  //left turn
  pwm_valA = max_pwm ;
  pwm_valB = fabs(msg->angular.z);
}
