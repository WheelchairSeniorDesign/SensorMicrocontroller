//
// Created by Vu, Nam on 11/20/24.
//

#ifndef BIOMETRICFUNCTIONS_H
#define BIOMETRICFUNCTIONS_H

#include <Arduino.h>
#include <Adafruit_Fingerprint.h>

/**
 * Initializes the capacitative fingerprint sensor with the given parameters
 * @param fp - An instance of the Adafruit_Fingerprint class for the fingerprint sensor
 * @param i2c_addr - The I2C address of the fingerprint sensor in hex
 */
void fpInit(Adafruit_Fingerprint &fp, uint8_t i2c_addr);

/**
*
*/

#endif //BIOMETRICFUNCTIONS_H
