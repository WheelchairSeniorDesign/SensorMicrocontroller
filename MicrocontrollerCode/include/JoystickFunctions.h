//
// Created by Robbie on 11/18/24.
//

#ifndef JOYSTICK_JOYSTICKFUNCTIONS_H
#define JOYSTICK_JOYSTICKFUNCTIONS_H

#include <Adafruit_ADS1X15.h>

/**
 * Struct representing the reference speed and direction.
 */
struct RefSpeed {
    int8_t leftSpeed;      ///< Speed of the left wheel.
    int8_t rightSpeed;     ///< Speed of the right wheel.
};

/**
 * Struct representing the reference displace
 */
struct RefDisplacement {
    int8_t longDisp;      ///< Forward/Backward displacement with + indicating forward
    int8_t latDisp;     ///< Side to side displacement with + indicating right
};

/**
 * Reads a value from the joystick connected to the ADC and returns the reference speeds
 * @param adc An instance of the adc the joystick is connected to
 * @return A RefSpeed for the wheelchair containing the wheel speeds and the direction
 */
RefSpeed joystickToSpeed(Adafruit_ADS1115 &adc);

/**
 * Reads a value from the joystick connected to the ADC and returns the reference displacement
 * @param adc An instance of the adc the joystick is connected to
 * @return A RefDisplacement for the joystick
 */
RefDisplacement joystickToDisplacement(Adafruit_ADS1115 &adc);

template <typename T>
/**
 * Custom clamp function to keep a value between to values
 * @tparam T The type of the value to clamp.
 * @param value The value to clamp.
 * @param min The minimum value.
 * @param max The maximum value.
 * @return Returns the clamped value.
 */
T clamp(T value, T min, T max);

#endif //JOYSTICK_JOYSTICKFUNCTIONS_H
