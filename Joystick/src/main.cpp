#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "ADCFunctions.h"
#include "JoystickFunctions.h"
#include "UltrasonicFunctions.h"

#ifdef ROS
#include <microRosFunctions.h>

#endif

Adafruit_ADS1115 joystickAdc;	// Construct an ads1115


void setup(void) {
    Serial.begin(115200);

    #ifdef ROS
        const char* nodeName = "sensors_node";
        const char* topicName = "sensors";
        microRosSetup(1, nodeName, topicName);
    #endif

    while (!Serial) {
        delay(10);
    }

    adcInit(joystickAdc, 0x48); //default address

}

void loop() {

    refSpeed omegaRef = joystickToSpeed(joystickAdc);
    int16_t usDistance = ultrasonicDistance(joystickAdc, 2);

    Serial.print("Right Speed: ");
    Serial.println(omegaRef.rightSpeed);
    Serial.print("Left Speed: ");
    Serial.println(omegaRef.leftSpeed);
    Serial.print("Ultrasonic Distance: ");
    Serial.println(usDistance);

    #ifdef ROS
        transmitMsg(omegaRef);

    #elif DEBUG

    #endif

}


//Setup and loop functions for the second core
void setup1() {
    Serial.println("Hello from the second core!");
    //TODO move all ROS communication over to second core so one is just collecting data and the other is just transmitting
}

void loop1() {
    Serial.println("This is the second core loop!");
}