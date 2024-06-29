#pragma once

#include <Arduino.h>

/**
 * Default device ID is 0.
*/
#ifndef DEVICE_ID
#define DEVICE_ID 0
#endif

/**
 * Teensy
*/
#if DEVICE_ID == 1
    #define ID_Serial Serial
    #define ID_PIN_LED LED_BUILTIN
#endif

/**
 * Pico
*/
#if DEVICE_ID == 2
    #define ID_Serial Serial
    #define ID_PIN_LED 22

    // UART Serial2(8, 9, NC, NC);
#endif