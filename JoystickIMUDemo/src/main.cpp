
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// Micro ROS example at https://github.com/micro-ROS/micro_ros_platformio/blob/main/examples/micro-ros_publisher

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

Adafruit_ADS1115 adc;	// Construct an ads1115
Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

void setup(void) {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);
    while (!Serial) {
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }

    allocator = rcl_get_default_allocator();

    // Set the domain ID
    const size_t domain_id = 1; // Replace with your desired domain ID

    //create init_options
    RCCHECK(rclc_support_init(&support, domain_id, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_platformio_node_publisher"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    Serial.println("Adafruit ICM20948 test!");

    // Try to initialize IMU!
    if (!icm.begin_I2C()) {
        // if (!icm.begin_SPI(ICM_CS)) {
        // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

        Serial.println("Failed to find ICM20948 chip");
        while (1) {
            delay(10);
        }
    }


    Serial.println("ICM20948 Found!");
    // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
    Serial.print("Accelerometer range set to: ");
    switch (icm.getAccelRange()) {
        case ICM20948_ACCEL_RANGE_2_G:
            Serial.println("+-2G");
            break;
        case ICM20948_ACCEL_RANGE_4_G:
            Serial.println("+-4G");
            break;
        case ICM20948_ACCEL_RANGE_8_G:
            Serial.println("+-8G");
            break;
        case ICM20948_ACCEL_RANGE_16_G:
            Serial.println("+-16G");
            break;
    }
    Serial.println("OK");

    // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    Serial.print("Gyro range set to: ");
    switch (icm.getGyroRange()) {
        case ICM20948_GYRO_RANGE_250_DPS:
            Serial.println("250 degrees/s");
            break;
        case ICM20948_GYRO_RANGE_500_DPS:
            Serial.println("500 degrees/s");
            break;
        case ICM20948_GYRO_RANGE_1000_DPS:
            Serial.println("1000 degrees/s");
            break;
        case ICM20948_GYRO_RANGE_2000_DPS:
            Serial.println("2000 degrees/s");
            break;
    }

    //  icm.setAccelRateDivisor(4095);
    uint16_t accel_divisor = icm.getAccelRateDivisor();
    float accel_rate = 1125 / (1.0 + accel_divisor);

    Serial.print("Accelerometer data rate divisor set to: ");
    Serial.println(accel_divisor);
    Serial.print("Accelerometer data rate (Hz) is approximately: ");
    Serial.println(accel_rate);

    //  icm.setGyroRateDivisor(255);
    uint8_t gyro_divisor = icm.getGyroRateDivisor();
    float gyro_rate = 1100 / (1.0 + gyro_divisor);

    Serial.print("Gyro data rate divisor set to: ");
    Serial.println(gyro_divisor);
    Serial.print("Gyro data rate (Hz) is approximately: ");
    Serial.println(gyro_rate);

    // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
    Serial.print("Magnetometer data rate set to: ");
    switch (icm.getMagDataRate()) {
        case AK09916_MAG_DATARATE_SHUTDOWN:
            Serial.println("Shutdown");
            break;
        case AK09916_MAG_DATARATE_SINGLE:
            Serial.println("Single/One shot");
            break;
        case AK09916_MAG_DATARATE_10_HZ:
            Serial.println("10 Hz");
            break;
        case AK09916_MAG_DATARATE_20_HZ:
            Serial.println("20 Hz");
            break;
        case AK09916_MAG_DATARATE_50_HZ:
            Serial.println("50 Hz");
            break;
        case AK09916_MAG_DATARATE_100_HZ:
            Serial.println("100 Hz");
            break;
    }
    Serial.println();

    //Initialize ADC
    while(!adc.begin(0x48)){ // Initialize ads1115 at address 0x49
        Serial.println("Failed to find ADS1115 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("ADS1115 Found!");

}

void loop() {

    //  /* Get a new normalized sensor event */
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    //Get ADC data
    int16_t adc0, adc1, adc2, adc3;


    /*Serial.print("Temperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");

    *//* Display the results (acceleration is measured in m/s^2) *//*
    Serial.print("Accel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");

    Serial.print("Mag X: ");
    Serial.print(mag.magnetic.x);
    Serial.print(" \tY: ");
    Serial.print(mag.magnetic.y);
    Serial.print(" \tZ: ");
    Serial.print(mag.magnetic.z);
    Serial.println(" uT");

    *//* Display the results (acceleration is measured in m/s^2) *//*
    Serial.print("Gyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();
    */

    adc0 = adc.readADC_SingleEnded(0);
    adc1 = adc.readADC_SingleEnded(1);
    adc2 = adc.readADC_SingleEnded(2);
    adc3 = adc.readADC_SingleEnded(3);
    Serial.print("AIN0: "); Serial.println(adc0);
    Serial.print("AIN1: "); Serial.println(adc1);
    //Serial.print("AIN2: "); Serial.println(adc2);
    //Serial.print("AIN3: "); Serial.println(adc3);
    Serial.println();
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);



    //  Serial.print(temp.temperature);
    //
    //  Serial.print(",");
    //
    //  Serial.print(accel.acceleration.x);
    //  Serial.print(","); Serial.print(accel.acceleration.y);
    //  Serial.print(","); Serial.print(accel.acceleration.z);
    //
    //  Serial.print(",");
    //  Serial.print(gyro.gyro.x);
    //  Serial.print(","); Serial.print(gyro.gyro.y);
    //  Serial.print(","); Serial.print(gyro.gyro.z);
    //
    //  Serial.print(",");
    //  Serial.print(mag.magnetic.x);
    //  Serial.print(","); Serial.print(mag.magnetic.y);
    //  Serial.print(","); Serial.print(mag.magnetic.z);

    //  Serial.println();
    //
    //  delayMicroseconds(measurement_delay_us);
}