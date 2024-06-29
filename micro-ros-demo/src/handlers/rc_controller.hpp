/** ROS2 Node to change LED status whenever it sees a number greater than 10 */
#pragma once

#include <Arduino.h>
#include <Servo.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <support/helper.hpp>


#define MOTOR_PIN 0                                                               
#define SERVO_PIN 28

#define LEFT_BOUND 50
#define RIGHT_BOUND 135

#define ZERO_SPEED 90  
#define MAX_FORWARD_SPEED 122
#define MAX_BACKWARD_SPEED 56


namespace rc_controller
{

const size_t HANDLER_COUNT = 1;  

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel;
Servo servo;
Servo esc;

/** Function Prototypes */
void cmd_vel_callback(const void *);

/**
 * @brief Sets up all the data necessary for a node to run
 * 
 * @param support ros client library support structure
 */
void init_handlers(rclc_support_t &support, rcl_node_t &node) {
    // Configure local information
    RCL_UNUSED(support);  // prevent unused warning
    servo.attach(SERVO_PIN);
    esc.attach(MOTOR_PIN);

    // create subscriber for command velocity
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
}

/**
 * @brief Attaches our handlers to the executor, in this case both our subscribers
 * 
 * @param executor ros executor structure with executor info
 */
void attach_to_executor(rclc_executor_t &executor) {
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
}

/**
 * @brief Called every time the subscriber sees a new message
 * 
 * @param msg_in the message recieved by the subscriber 
 */
void cmd_vel_callback(const void *msg_in) {
    // read the speeds
    auto cmd_vel = (const geometry_msgs__msg__Twist *) msg_in;
    double linear = cmd_vel->linear.x;
    double angular = cmd_vel->angular.z;

    // map the speeds to motor values
    linear = map(linear, -1.0, 1.0, MAX_BACKWARD_SPEED, MAX_FORWARD_SPEED);
    angular = map(angular, -1.0, 1.0, LEFT_BOUND, RIGHT_BOUND);

    // drive the components
    esc.write((int)linear);
    servo.write((int)angular);
}

}