/** ROS2 GPS Wrapper. */
#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/nav_sat_fix.h>

#include <support/helper.hpp>

#define SERIAL_PORT Serial1
#define ONLY_PUBLISH_NEW_DATA false

#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_WEEK 604800
#define SECONDS_IN_MONTH 2629743
#define SECONDS_IN_YEAR 31556926

namespace gps
{

const size_t HANDLER_COUNT = 1;  // only handler is a single timer

rcl_publisher_t data_pub;
sensor_msgs__msg__NavSatFix gps_data;
rcl_timer_t timer;

TinyGPSPlus gps;


/** Function Prototypes */
void timer_callback(rcl_timer_t *, int64_t);

/**
 * @brief Sets up all the data necessary for a node to run
 * 
 * @param support ros client library support structure
 */
void init_handlers(rclc_support_t &support, rcl_node_t &node) {
    // Configure local information
    SERIAL_PORT.begin(9600);

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &data_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "gps0/raw"));

    // create timer to trigger publisher
    const unsigned int delta_t = 500;  // [ms] time between timer ticks
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(delta_t),  // time between publishes
        timer_callback));
}

/**
 * @brief Attaches our handlers to the executor, like our timer
 * 
 * @param executor ros executor structure with executor info
 */
void attach_to_executor(rclc_executor_t &executor) {
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

/*
int32_t date_to_epoch_time() {
    gps.date.day();
    gps.date.month();
    gps.date.year();
    gps.time.hour() * SECONDS_IN_HOUR
    + gps.time.minute() * 60
    + gps.time.second();
    return 0;
}
*/

/**
 * @brief Called at a set interval, publishes a message every tick
 * 
 * @param timer ros2 timer object
 * @param last_call_time the last time this function was called
 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        while (SERIAL_PORT.available() > 0) {
            if (gps.encode(SERIAL_PORT.read())) {  // returns true when encode is done
                if (gps.location.isUpdated() || !ONLY_PUBLISH_NEW_DATA) {
                    gps_data.latitude = gps.location.lat();
                    gps_data.longitude = gps.location.lng();
                    gps_data.altitude = gps.altitude.meters();
                    gps_data.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN; 
                    
                    // calculate time since epoch
                    //gps_data.header.stamp.sec;
                    RCSOFTCHECK(rcl_publish(&data_pub, &gps_data, NULL));\
                }       
            }
        }

        
    }
}

}