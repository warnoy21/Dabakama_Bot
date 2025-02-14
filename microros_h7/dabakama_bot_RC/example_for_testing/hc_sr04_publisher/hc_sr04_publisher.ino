/*
 * Description: Microros for Portenta H7 to get data from the HC_SR04 sensor. The node is "micro_ros_arduino_node"
 * and subscribes to "motor_value_topic" for motor inputs.
 *              
 * 
 * Version 1.0      Date: Februart 12,2025 Initial Creation         Made by: Aaron Gumba
 * 
 */
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include "hc_sr04_publisher.h"

#define LED_PIN 13

rcl_publisher_t obstacle_distance_publisher;
std_msgs__msg__String distance_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

    // Initialize micro-ROS
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

    // Create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &obstacle_distance_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "micro_ros_arduino_node_publisher"));

    // Create timer (trigger every 500ms)
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(500),
        obstacle_distance_callback));

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
