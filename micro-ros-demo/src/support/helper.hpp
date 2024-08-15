#pragma once

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <support/identity.hpp>  // included so files only need to import helper

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if(temp_rc != RCL_RET_OK){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

double map_range(double num, double in_start, double in_end, double out_start, double out_end) {
    // uses basic y = mx+b formula to map number from in range to out range
    double slope = (out_end - out_start) / (in_end - in_start);
    return out_start + slope * (num - in_start);
}