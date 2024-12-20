//
// Created by Robbie on 11/20/24.
//
#ifdef ROS
#include "microRosFunctions.h"
#include "JoystickFunctions.h"
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <wheelchair_sensor_msgs/msg/sensors.h>

rcl_publisher_t publisher;
wheelchair_sensor_msgs__msg__Sensors msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while(1) {
        delay(100);
    }
}

void timer_callback(rcl_timer_t * inputTimer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }
}

void microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* topicName){
    set_microros_serial_transports(Serial);
    delay(2000);
    allocator = rcl_get_default_allocator();

    // Set the domain ID
    const size_t domain_id = 7; // Replace with your desired domain ID

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Sensors),
            topicName));

    // create timer,
    //unsigned int timer_timeout = 1;
    RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.left_speed = 0;
    msg.right_speed = 0;
}

void transmitMsg(refSpeed omegaRef){
    msg.left_speed = omegaRef.leftSpeed;
    msg.right_speed = omegaRef.rightSpeed;

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
#endif