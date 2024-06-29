#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

#include <support/helper.hpp>


#define IMUEXTRA_TOPIC_NAME "imu"
#define IMUEXTRA_UPDATE_PERIOD_MS 50

#define ID_IMU4_WIRE Wire2
#define ID_IMU5_WIRE Wire2
#define ID_IMU6_WIRE Wire1

#define ID_IMU4_ADDR 0x68
#define ID_IMU5_ADDR 0x69
#define ID_IMU6_ADDR 0x68

class IMUEXTRA {
    private:
    uint8_t imu_id;
    Adafruit_MPU6050 mpu;

    rcl_publisher_t publisher;
    sensor_msgs__msg__Imu msg;

    public:
    IMUEXTRA(u_int8_t id, TwoWire &wire, uint8_t addr) {
        this->imu_id = id;

        // Initialize IMU
        mpu.begin(addr, &wire, id);

        mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
        mpu.setMotionDetectionThreshold(1);
        mpu.setMotionDetectionDuration(20);
        mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
        mpu.setInterruptPinPolarity(true);
        mpu.setMotionInterrupt(true);
    }

    rcl_ret_t measure() {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        //TODO: Calculate orientation
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 0;

        msg.angular_velocity.x = g.gyro.x;
        msg.angular_velocity.y = g.gyro.y;
        msg.angular_velocity.z = g.gyro.z;

        msg.linear_acceleration.x = a.acceleration.x;
        msg.linear_acceleration.y = a.acceleration.y;
        msg.linear_acceleration.z = a.acceleration.z;

        //TODO: Fill in covariance

        // Publish message
        return rcl_publish(&publisher, &msg, nullptr);
    }

    void init_handlers(rclc_support_t &support, rcl_node_t &node) {
        // create publisher
        RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            (IMUEXTRA_TOPIC_NAME + String(imu_id) + "/raw").c_str()));
    }
};

namespace imuextra {
    const size_t HANDLER_COUNT = 1;  // one timer

    rcl_timer_t timer;

    IMUEXTRA imu4(0, ID_IMU4_WIRE, ID_IMU4_ADDR);
    IMUEXTRA imu5(1, ID_IMU5_WIRE, ID_IMU5_ADDR);
    IMUEXTRA imu6(2, ID_IMU6_WIRE, ID_IMU6_ADDR);

    void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL) {
            imu4.measure();
            imu5.measure();
            imu6.measure();
        }
    }

    /**
     * @brief Sets up all the data necessary for a node to run
     * 
     * @param support ros client library support structure
     */
    void init_handlers(rclc_support_t &support, rcl_node_t &node) {
        imu4.init_handlers(support, node);
        imu5.init_handlers(support, node);
        imu6.init_handlers(support, node);

        // Create timer
        const unsigned int delta_t = IMUEXTRA_UPDATE_PERIOD_MS;
        RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(delta_t),
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
};