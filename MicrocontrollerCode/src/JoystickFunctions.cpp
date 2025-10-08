//
// Created by Robbie on 11/18/24.
//

#include "JoystickFunctions.h"
#include <Adafruit_ADS1X15.h>
#include <algorithm>

int diffParam = 30;
int deadzoneParam = 30;

RefDisplacement joystickToDisplacement(Adafruit_ADS1115 &adc){
    int forwardJoystick = adc.readADC_SingleEnded(0); //a0 is forward/backward
    int sidewaysJoystick = adc.readADC_SingleEnded(1); //a1 is left/right

    RefDisplacement displacements;
    /*
     * Joystick middle values: ~8500
     * a0 middle value: ~8500
     * a1 middle value: ~8300
     * a0 deadzone 10000 - 6500
     * a1 deadzone 11000 - 6000
     * Joystick Min: 0
     * Joystick Max: 17390
     * Output is a value -100 to 100 for the speed of the motor
     */

    //Converting the speeds so they start around 0 and then go positive and negative
    displacements.longDisp = forwardJoystick - (8500+4400); //The second value is used to zero it out when the ADC gain is set to 0 instead of the default (2/3)
    displacements.latDisp = sidewaysJoystick - (8400+4400);
    return displacements;
}

RefSpeed joystickToSpeed(Adafruit_ADS1115 &adc){
    int forwardJoystick = adc.readADC_SingleEnded(0); //a0 is forward/backward
    int sidewaysJoystick = adc.readADC_SingleEnded(1); //a1 is left/right

    /*
     * Joystick middle values: ~8500
     * a0 middle value: ~8500
     * a1 middle value: ~8300
     * a0 deadzone 10000 - 6500
     * a1 deadzone 11000 - 6000
     * Joystick Min: 0
     * Joystick Max: 17390
     * Output is a value -100 to 100 for the speed of the motor
     */

    //Converting the speeds so they start around 0 and then go positive and negative
    forwardJoystick = forwardJoystick - (8500+4400); //The second value is used to zero it out when the ADC gain is set to 0 instead of the default (2/3)
    sidewaysJoystick = sidewaysJoystick - (8400+4400);

//    Serial.print("Forward joystick: ");
//    Serial.println(forwardJoystick);
//    Serial.print("Sideways joystick: ");
//    Serial.println(sidewaysJoystick);

    //setting the speeds
    RefSpeed speeds{};
    const float MAX_INPUT = 13000.0f;
    const float BACKWARD_X_THRESH = 0.65f;  // 0.0…1.0

    // 1) normalize
    float x = constrain(sidewaysJoystick / MAX_INPUT, -1.0f, 1.0f);
    float y = constrain(forwardJoystick  / MAX_INPUT, -1.0f, 1.0f);

    // 1b) pure backward shortcut
    if (fabsf(x) < BACKWARD_X_THRESH && y < 0.0f) {
        int8_t rev = (int8_t)roundf(y * 100.0f);
        speeds.leftSpeed  = rev;
        speeds.rightSpeed = rev;

        //Deadzone
        if(speeds.leftSpeed < deadzoneParam && speeds.leftSpeed > -deadzoneParam && speeds.rightSpeed < deadzoneParam && speeds.rightSpeed > -deadzoneParam){
            speeds.leftSpeed = 0;
            speeds.rightSpeed = 0;
            return speeds;
        }

        return speeds;
    }

    // 2) magnitude clamp
    float mag = hypotf(x, y);
    mag = constrain(mag, 0.0f, 1.0f);

    // 3+4) decide pivot vs mix
    float outer, inner;
    if (fabsf(x) > 0.0f && y <= 0.0f) {
        // any sideways + backward → pivot (inner=0, outer=1)
        outer = 1.0f;
        inner = 0.0f;
    } else {
        // forward or straight back (x==0) → smooth mix
        float angle     = atan2f(fabsf(x), fabsf(y));
        float turn_prop = angle / (M_PI_2);
        outer = 1.0f;
        inner = 1.0f - turn_prop;
    }

    // 5) assign inner/outer to left/right
    float left_f  = (x >= 0.0f) ? inner : outer;
    float right_f = (x >= 0.0f) ? outer : inner;

    // 6) direction: any sideways → forward pivot; otherwise follow y
    int dir = (fabsf(x) > 0.0f) ? +1 : (y >= 0.0f ? +1 : -1);

    // 7) apply magnitude & direction
    left_f  *= mag * dir;
    right_f *= mag * dir;

    // 8) clamp & scale to –100…+100
    left_f  = constrain(left_f,  -1.0f, 1.0f);
    right_f = constrain(right_f, -1.0f, 1.0f);
    speeds.leftSpeed  = (int8_t)roundf(left_f  * 100.0f);
    speeds.rightSpeed = (int8_t)roundf(right_f * 100.0f);

    //Deadzone
    if(speeds.leftSpeed < deadzoneParam && speeds.leftSpeed > -deadzoneParam && speeds.rightSpeed < deadzoneParam && speeds.rightSpeed > -deadzoneParam){
        speeds.leftSpeed = 0;
        speeds.rightSpeed = 0;
        return speeds;
    }

    //Middle zone to have the same speed
    int diff = abs(speeds.leftSpeed - speeds.rightSpeed);
    if(diff > -diffParam && diff < diffParam){
        // Check if the speeds are positive or negative and set accordingly
        if (speeds.leftSpeed > 0 && speeds.rightSpeed > 0) {
            // Both speeds are positive, set the smaller one to the larger one
            if (speeds.leftSpeed < speeds.rightSpeed) {
                speeds.leftSpeed = speeds.rightSpeed;
            } else {
                speeds.rightSpeed = speeds.leftSpeed;
            }
        } else if (speeds.leftSpeed < 0 && speeds.rightSpeed < 0) {
            // Both speeds are negative, set the larger one to the smaller one
            if (speeds.leftSpeed > speeds.rightSpeed) {
                speeds.leftSpeed = speeds.rightSpeed;
            } else {
                speeds.rightSpeed = speeds.leftSpeed;
            }
        }
    }

    return speeds;
}

template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
