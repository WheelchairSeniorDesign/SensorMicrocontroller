//
// Created by Robbie on 11/20/24.
//
#if defined(ROS) || defined(ROS_DEBUG)
#include "microRosFunctions.h"
#include "JoystickFunctions.h"
#include "FanFunctions.h"
#include "LightFunctions.h"
#include "LidarFunctions.h"
#include <Arduino.h>
#include <hardware/watchdog.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <wheelchair_sensor_msgs/msg/sensors.h>
#include <wheelchair_sensor_msgs/msg/fingerprint.h>
#include <wheelchair_sensor_msgs/msg/fan_speed.h>
#include <wheelchair_sensor_msgs/msg/light.h>
#include <wheelchair_sensor_msgs/msg/lidar.h>
#include <wheelchair_sensor_msgs/msg/sensor_error.h>


#ifdef ROS_DEBUG
#include <wheelchair_sensor_msgs/msg/ref_speed.h>
#endif


rcl_publisher_t sensorPublisher;
rcl_publisher_t fingerprintPublisher;
rcl_publisher_t errorPublisher;

rcl_subscription_t fanSubscriber;
rcl_subscription_t lightSubscriber;
rcl_subscription_t lidarSubscriber;

wheelchair_sensor_msgs__msg__FanSpeed fanMsg;
wheelchair_sensor_msgs__msg__Light lightMsg;
wheelchair_sensor_msgs__msg__Lidar lidarMsg;
#ifdef ROS
wheelchair_sensor_msgs__msg__Sensors sensorMsg;
#elif ROS_DEBUG
wheelchair_sensor_msgs__msg__RefSpeed msg;
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

boolean agent_connected = false;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
    while (1) {
        delay(100);
    }
}

void timer_callback(rcl_timer_t *inputTimer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
#ifdef ROS
        rcl_publish(&sensorPublisher, &sensorMsg, NULL);
#elif ROS_DEBUG
        RCSOFTCHECK(rcl_publish(&sensorPublisher, &msg, NULL));
#endif
    }
}

bool create_entities(){
    allocator = rcl_get_default_allocator();

    // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "sensors_node", "", &support));

  #ifdef ROS
    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &sensorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Sensors),
        "sensors"));
    // Create fingerprint publisher
    RCCHECK(rclc_publisher_init_default(
        &fingerprintPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Fingerprint),
        "fingerprint"));

    //Create error publisher
    RCCHECK(rclc_publisher_init_default(
        &errorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, SensorError),
        "sensor_error"));

    // create timer,
    //unsigned int timer_timeout = 1;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(10),
        timer_callback));

#elif ROS_DEBUG
    // create publisher
RCCHECK(rclc_publisher_init_best_effort(
        &sensorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, RefSpeed),
        topicName));
#endif

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &fanSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, FanSpeed),
        "fan_duty_cycles"));

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &lightSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Light),
        "light"));

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &lidarSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Lidar),
        "lidar"));

    // create executor
    //Number of handles = # timers + # subscriptions + # clients + # services
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &fanSubscriber, &fanMsg, &fan_subscription_callback, ON_NEW_DATA));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &lightSubscriber, &lightMsg, &light_subscription_callback, ON_NEW_DATA
        ));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &lidarSubscriber, &lidarMsg, &lidar_subscription_callback, ON_NEW_DATA
        ));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    #ifdef ROS
    sensorMsg.left_speed = 0;
    sensorMsg.right_speed = 0;
#elif ROS_DEBUG
    msg.left_speed = 0;
    msg.right_speed = 0;
#endif

    state = WAITING_AGENT;

    return true;

}

void destroy_entities(){
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCCHECK(rcl_subscription_fini(&fanSubscriber, &node));
    RCCHECK(rcl_subscription_fini(&lidarSubscriber, &node));
    RCCHECK(rcl_subscription_fini(&lightSubscriber, &node));
    RCCHECK(rcl_publisher_fini(&sensorPublisher, &node));
    RCCHECK(rcl_publisher_fini(&fingerprintPublisher, &node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
}

void microRosTick(){
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                               state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED
                                                                                        : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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

//TODO add the fan subscriber
#ifdef ROS
boolean microRosSetup(unsigned int timer_timeout, const char *nodeName, const char *sensorTopicName,
                   const char *fingerprintTopicName) {
#elif ROS_DEBUG
    boolean microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* topicName){
#endif
    set_microros_serial_transports(Serial);
    delay(2000);

//    // Check for agent BEFORE proceeding
//    if (!rmw_uros_ping_agent(100, 3)) { // 100ms timeout, 3 retries
//        printf("Micro-ROS agent not found — skipping setup.\n");
//        return false;
//    }

    allocator = rcl_get_default_allocator();

    // Set the domain ID
    //const size_t domain_id = 7; // Replace with your desired domain ID

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

#ifdef ROS
    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &sensorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Sensors),
        sensorTopicName));
    // Create fingerprint publisher
    RCCHECK(rclc_publisher_init_default(
        &fingerprintPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Fingerprint),
        fingerprintTopicName));

    // create timer,
    //unsigned int timer_timeout = 1;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

#elif ROS_DEBUG
    // create publisher
RCCHECK(rclc_publisher_init_best_effort(
        &sensorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, RefSpeed),
        topicName));
#endif

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &fanSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, FanSpeed),
        "fan_duty_cycles"));

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &lightSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Light),
        "light"));

    //Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &lidarSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Lidar),
        "lidar"));


    // create executor
    //Number of handles = # timers + # subscriptions + # clients + # services
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &fanSubscriber, &fanMsg, &fan_subscription_callback, ON_NEW_DATA));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &lightSubscriber, &lightMsg, &light_subscription_callback, ON_NEW_DATA
        ));
    RCCHECK(
        rclc_executor_add_subscription(&executor, &lidarSubscriber, &lidarMsg, &lidar_subscription_callback, ON_NEW_DATA
        ));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));


#ifdef ROS
    sensorMsg.left_speed = 0;
    sensorMsg.right_speed = 0;
#elif ROS_DEBUG
    msg.left_speed = 0;
    msg.right_speed = 0;
#endif
    return true;
}


void publishFingerprint(uint8_t fingerprintID) {
        wheelchair_sensor_msgs__msg__Fingerprint fingerprintMsg;
        fingerprintMsg.user_id = fingerprintID;
        RCSOFTCHECK(rcl_publish(&fingerprintPublisher, &fingerprintMsg, NULL));
    }

void publishError(const bool joystick_adc_error, const bool ultrasonic_adc_error, const bool fingerprint_error, const bool imu_error) {
        wheelchair_sensor_msgs__msg__SensorError sensorErrorMsg;
        sensorErrorMsg.joystick_adc_error = joystick_adc_error;
        sensorErrorMsg.ultrasonic_adc_error = ultrasonic_adc_error;
        sensorErrorMsg.fingerprint_error = fingerprint_error;
        sensorErrorMsg.imu_error = imu_error;
        RCSOFTCHECK(rcl_publish(&errorPublisher, &sensorErrorMsg, NULL));
    }

static void fan_subscription_callback(const void *msgin) {
        const auto *msg = (const wheelchair_sensor_msgs__msg__FanSpeed *) msgin;
        FanDutyCycles duty_cycles{};
        duty_cycles.fan_0_duty_cycle = msg->fan_percent_0;
        duty_cycles.fan_1_duty_cycle = msg->fan_percent_1;
        duty_cycles.fan_2_duty_cycle = msg->fan_percent_2;
        duty_cycles.fan_3_duty_cycle = msg->fan_percent_3;
        setAllFans(duty_cycles);
    }

static void light_subscription_callback(const void *msgin) {
        const auto *msg = (const wheelchair_sensor_msgs__msg__Light *) msgin;
        const int lightState = msg->state;
        setLight(lightState);
    }

static void lidar_subscription_callback(const void *msgin){
        const auto *msg = (const wheelchair_sensor_msgs__msg__Lidar *) msgin;
        if(msg->state == 0){
            lidarState(false);
        } else if(msg->state == 1){
            lidarState(true);
        }
    }

#ifdef ROS
void transmitMsg(RefDisplacement thetaRef, RefSpeed omegaRef, USData ultrasonicData, PIRSensors pirSensors, FanSpeeds fanSpeeds, IMUData imuData) {
    sensorMsg.long_disp = thetaRef.longDisp;
    sensorMsg.lat_disp = thetaRef.latDisp;
    sensorMsg.left_speed = omegaRef.leftSpeed;
    sensorMsg.right_speed = omegaRef.rightSpeed;
    sensorMsg.ultrasonic_front_0 = ultrasonicData.us_front_0;
    sensorMsg.ultrasonic_front_1 = ultrasonicData.us_front_1;
    sensorMsg.ultrasonic_back = ultrasonicData.us_back;
    sensorMsg.ultrasonic_left = ultrasonicData.us_left;
    sensorMsg.ultrasonic_right = ultrasonicData.us_right;
    sensorMsg.pir_front = pirSensors.pir0;
    sensorMsg.pir_back = pirSensors.pir1;
    sensorMsg.pir_left = pirSensors.pir2;
    sensorMsg.pir_right = pirSensors.pir3;
    sensorMsg.fan_speed_0 = fanSpeeds.fan_speed_0;
    sensorMsg.fan_speed_1 = fanSpeeds.fan_speed_1;
    sensorMsg.fan_speed_2 = fanSpeeds.fan_speed_2;
    sensorMsg.fan_speed_3 = fanSpeeds.fan_speed_3;
    sensorMsg.linear_acceleration_x = imuData.accel_x;
    sensorMsg.linear_acceleration_y = imuData.accel_y;
    sensorMsg.linear_acceleration_z = imuData.accel_z;
    sensorMsg.angular_velocity_x = imuData.gyro_x;
    sensorMsg.angular_velocity_y = imuData.gyro_y;
    sensorMsg.angular_velocity_z = imuData.gyro_z;
    sensorMsg.magnetic_field_x = imuData.mag_x;
    sensorMsg.magnetic_field_y = imuData.mag_y;
    sensorMsg.magnetic_field_z = imuData.mag_z;

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}



#elif ROS_DEBUG

void transmitMsg(RefDisplacement thetaRef, RefSpeed omegaRef){
    msg.left_speed = omegaRef.leftSpeed;
    msg.right_speed = omegaRef.rightSpeed;
    msg.long_disp = thetaRef.longDisp;
    msg.lat_disp = thetaRef.latDisp;

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

#endif

#endif
