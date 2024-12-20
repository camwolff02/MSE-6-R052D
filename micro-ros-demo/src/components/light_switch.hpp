/** ROS2 Node to change LED status whenever it sees a number greater than 10 */
#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <support/helper.hpp>


namespace light_switch
{

const size_t HANDLER_COUNT = 2;  // two handlers, one for each subscription 

rcl_subscription_t on_subscriber;
rcl_subscription_t off_subscriber;
std_msgs__msg__Int32 on_msg;
std_msgs__msg__Int32 off_msg;

/** Function Prototypes */
void on_switch_callback(const void *);
void off_switch_callback(const void *);

/**
 * @brief Sets up all the data necessary for a node to run
 * 
 * @param support ros client library support structure
 */
void init_handlers(rclc_support_t &support, rcl_node_t &node) {
    // Configure local information
    RCL_UNUSED(support);  // prevent unused warning
    pinMode(ID_PIN_LED, OUTPUT);

    // create first subscriber for on switch
    RCCHECK(rclc_subscription_init_default(
        &on_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "daytime"));

    // create second subscriber for off switch
    RCCHECK(rclc_subscription_init_default(
        &off_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "daytime"));
}

/**
 * @brief Attaches our handlers to the executor, in this case both our subscribers
 * 
 * @param executor ros executor structure with executor info
 */
void attach_to_executor(rclc_executor_t &executor) {
    RCCHECK(rclc_executor_add_subscription(&executor, &on_subscriber, &on_msg, &on_switch_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &off_subscriber, &off_msg, &off_switch_callback, ON_NEW_DATA));
}

/**
 * @brief Called every time the subscriber sees a new message
 * 
 * @param msg_in the message recieved by the subscriber 
 */
void on_switch_callback(const void *msg_in) {
    int daytime = ((const std_msgs__msg__Int32 *)msg_in)->data;

    if (!daytime) {
        digitalWrite(ID_PIN_LED, HIGH);
    }
}

/**
 * @brief Called every time the subscriber sees a new message
 * 
 * @param msg_in the message recieved by the subscriber 
 */
void off_switch_callback(const void *msg_in) {
    int daytime = ((const std_msgs__msg__Int32 *)msg_in)->data;

    if (daytime) {
        digitalWrite(ID_PIN_LED, LOW);
    }
}

}