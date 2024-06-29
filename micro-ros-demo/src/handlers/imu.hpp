#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/vector3.h>

#include <support/helper.hpp>

#include <BMI088.h>

#define IMU_TOPIC_NAME "imu"
#define IMU_UPDATE_PERIOD_MS 50

#define ID_IMU1_Wire Wire
#define ID_IMU1_ACCL_ADDR 0x18
#define ID_IMU1_GYRO_ADDR 0x68

#define ID_IMU2_Wire Wire1
#define ID_IMU2_ACCL_ADDR 0x18
#define ID_IMU2_GYRO_ADDR 0x68

#define ID_IMU3_Wire Wire2
#define ID_IMU3_ACCL_ADDR 0x18
#define ID_IMU3_GYRO_ADDR 0x68


class IMU {
    private:
    uint8_t imu_id;
    Bmi088Accel accl;
    Bmi088Gyro gyro;

    rcl_publisher_t publisher;
    sensor_msgs__msg__Imu msg;

    public:
    IMU(u_int8_t id, TwoWire &wire, uint8_t accl_addr, uint8_t gyro_addr) 
    : accl(wire, accl_addr), gyro(wire, gyro_addr) {
        this->imu_id = id;

        // Initialize IMU
        accl.begin();
        gyro.begin();
    }

    rcl_ret_t measure() {
        // Read data from sensors
        accl.readSensor();
        gyro.readSensor();

        //TODO: Calculate orientation
        msg.orientation.x = 0;
        msg.orientation.y = 0;
        msg.orientation.z = 0;
        msg.orientation.w = 0;

        msg.angular_velocity.x = gyro.getGyroX_rads();
        msg.angular_velocity.y = gyro.getGyroY_rads();
        msg.angular_velocity.z = gyro.getGyroZ_rads();

        msg.linear_acceleration.x = accl.getAccelX_mss();
        msg.linear_acceleration.y = accl.getAccelY_mss();
        msg.linear_acceleration.z = accl.getAccelZ_mss();

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
            (IMU_TOPIC_NAME + String(imu_id) + "/raw").c_str()));
    }
};

namespace imu {
    const size_t HANDLER_COUNT = 1;  // one timer

    rcl_timer_t timer;

    IMU imu0(0, ID_IMU1_Wire, ID_IMU1_ACCL_ADDR, ID_IMU1_GYRO_ADDR);
    IMU imu1(1, ID_IMU2_Wire, ID_IMU2_ACCL_ADDR, ID_IMU2_GYRO_ADDR);
    IMU imu2(2, ID_IMU3_Wire, ID_IMU3_ACCL_ADDR, ID_IMU3_GYRO_ADDR);

    void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL) {
            imu0.measure();
            imu1.measure();
            imu2.measure();
        }
    }

    /**
     * @brief Sets up all the data necessary for a node to run
     * 
     * @param support ros client library support structure
     */
    void init_handlers(rclc_support_t &support, rcl_node_t &node) {
        imu0.init_handlers(support, node);
        imu1.init_handlers(support, node);
        imu2.init_handlers(support, node);

        // Create timer
        const unsigned int delta_t = IMU_UPDATE_PERIOD_MS;
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