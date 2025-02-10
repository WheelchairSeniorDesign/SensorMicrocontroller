//
// Created by Robbie on 11/20/24.
//

#ifndef JOYSTICK_MICROROSFUNCTIONS_H
#define JOYSTICK_MICROROSFUNCTIONS_H

#include "JoystickFunctions.h"
#include "UltrasonicFunctions.h"
#include "PIRFunctions.h"
#include "FanFunctions.h"
#include "IMUFunctions.h"

#ifdef ROS
/**
 * Sets up microROS communication
 * @param timerValue The transmission period
 * @param nodeName The name of the node
 * @param sensorTopicName The name of the topic
 * @param fingerprintTopicName
 */
void microRosSetup(unsigned int timerValue, const char* nodeName, const char* sensorTopicName, const char* fingerprintTopicName);
#elif ROS_DEBUG
/**
 * Sets up microROS communication
 * @param timerValue The transmission period
 * @param nodeName The name of the node
 * @param topicName The name of the topic
 */
void microRosSetup(unsigned int timerValue, const char* nodeName, const char* topicName);
#endif

#ifdef ROS

void transmitMsg(RefSpeed omegaRef, USData ultrasonicData, PIRSensors pirSensors, FanSpeeds fanSpeeds, IMUData imuData);

void publishFingerprint(uint8_t fingerprintID);

#elif ROS_DEBUG

/**
 * Transmits the message over ROS
 * @param omegaRef The referenceSpeed struct to transmit
 */
void transmitMsg(RefSpeed omegaRef);

#endif

#endif //JOYSTICK_MICROROSFUNCTIONS_H
