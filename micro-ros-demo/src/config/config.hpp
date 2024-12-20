// This file is written a little weird because it is a config file, but
// if you follow the format, add a #ifdef for each of your build configurations,
// and put your functions in the right place, then it should all work!
#pragma once

// default imports
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

// Import all your c++ files here
#include <components/daylight_sensor.hpp>
#include <components/light_switch.hpp>
// #include <components/rc_controller.hpp>
#include <components/imu.hpp>

// Choose your node names here
#define NODE_NAME "micro_ros_handler"


// Put the handler counts for all your nodes here
const size_t HANDLER_COUNT = (
    daylight_sensor::HANDLER_COUNT + 
    light_switch::HANDLER_COUNT + 
    // rc_controller::HANDLER_COUNT + 
    imu::HANDLER_COUNT +
0);

/**
 * @brief Put the initializers for all your nodes here
 * 
 * @param support support struct to be passed to initializers
 */
void init_all_handlers(rclc_support_t &support, rcl_node_t &node) {
    daylight_sensor::init_handlers(support, node);
    light_switch::init_handlers(support, node);
    // rc_controller::init_handlers(support, node);
    imu::init_handlers(support, node);
}

/**
 * @brief Put the executor attachers for all your nodes here
 * 
 * @param executor executor struct to be passed to executors
 */
void attach_all_to_executor(rclc_executor_t &executor) {
    daylight_sensor::attach_to_executor(executor);
    light_switch::attach_to_executor(executor);
    // rc_controller::attach_to_executor(executor);
    imu::attach_to_executor(executor);
}