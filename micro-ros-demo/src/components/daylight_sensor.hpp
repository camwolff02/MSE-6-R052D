/** ROS2 Node to simulate daylight sensor. 
 * Publishes "0" for night, and "1" for day */
#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <support/helper.hpp>


namespace daylight_sensor
{

const size_t HANDLER_COUNT = 1;  // only handler is a single timer

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_timer_t timer;


/** Function Prototypes */
void timer_callback(rcl_timer_t *, int64_t);

/**
 * @brief Sets up all the data necessary for a node to run
 * 
 * @param support ros client library support structure
 */
void init_handlers(rclc_support_t &support, rcl_node_t &node) {
    // Configure local information
    msg.data = 0;

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "daytime"));

    // create timer to trigger publisher
    const unsigned int delta_t = 1000;  // [ms] time between timer ticks
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

/**
 * @brief Called at a set interval, publishes a message every tick
 * 
 * @param timer ros2 timer object
 * @param last_call_time the last time this function was called
 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data = !msg.data;
    }
}

}