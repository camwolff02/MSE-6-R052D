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
#define SERVO_PIN 22

#define LEFT_BOUND 50
#define RIGHT_BOUND 135

#define ZERO_SPEED 90  
#define MAX_FORWARD_SPEED 122
#define MAX_BACKWARD_SPEED 56


namespace rc_controller
{

const size_t HANDLER_COUNT = 3;  

rcl_subscription_t subscriber;
rcl_timer_t motor_timer;
rcl_timer_t steering_timer;
geometry_msgs__msg__Twist cmd_vel;

Servo steering;
Servo esc;
int linear;
int angular;
unsigned int steer_period;

/** Function Prototypes */
void cmd_vel_callback(const void *);
void motor_timer_callback(rcl_timer_t *, int64_t);
void steering_timer_callback(rcl_timer_t *, int64_t);

/**
 * @brief Sets up all the data necessary for a node to run
 * 
 * @param support ros client library support structure
 */
void init_handlers(rclc_support_t &support, rcl_node_t &node) {
    // Configure local information
    RCL_UNUSED(support);  // prevent unused warning
    steering.attach(SERVO_PIN);
    esc.attach(MOTOR_PIN);
    linear = 0;
    angular = (LEFT_BOUND + RIGHT_BOUND) / 2;

    // "teach" throttle range
    esc.write(MAX_FORWARD_SPEED);
    esc.write(ZERO_SPEED);

    // create subscriber for command velocity
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // create timers for all of our effectors
    const unsigned int frequency = 10000;
    RCCHECK(rclc_timer_init_default(
        &motor_timer,
        &support,
        RCL_MS_TO_NS(1/frequency),  // time between publishes
        motor_timer_callback));

    steer_period = 500;
    RCCHECK(rclc_timer_init_default(
        &steering_timer,
        &support,
        RCL_MS_TO_NS(steer_period),  // time between publishes
        steering_timer_callback));

    // allow time for throttle teaching
    delay(2000);
}

/**
 * @brief Attaches our handlers to the executor, in this case both our subscribers
 * 
 * @param executor ros executor structure with executor info
 */
void attach_to_executor(rclc_executor_t &executor) {
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &motor_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &steering_timer));
}

/**
 * @brief Called every time the subscriber sees a new message
 * 
 * @param msg_in the message recieved by the subscriber 
 */
void cmd_vel_callback(const void *msg_in) {
    // read the speeds
    auto cmd_vel = (const geometry_msgs__msg__Twist *) msg_in;
    double linear_vel = cmd_vel->linear.x;
    double angular_vel = cmd_vel->angular.z;

    // map the speeds to motor values
    linear = map_range(linear_vel, -1.0, 1.0, MAX_BACKWARD_SPEED, MAX_FORWARD_SPEED);
    angular = map_range(angular_vel, 1.0, -1.0, LEFT_BOUND, RIGHT_BOUND);
}

void motor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        esc.write(linear);
    }
}

void steering_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        int curr_pos = steering.read();
        int increment = 1;

        if (curr_pos != angular) {
            steering.write(curr_pos + increment * (curr_pos < angular? 1 : -1));
            delay(steer_period);
        }

        return;
        int dir = steering.read() < angular? 1 : -1;
        for (int pos = steering.read(); pos != angular; pos += dir) {
            steering.write(pos);
            delay(10);
        }
    }
}


}