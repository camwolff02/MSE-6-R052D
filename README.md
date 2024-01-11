# MSE-6-R052D
ROS2 Mouse Droid

Parts:
- [Micro Servo](https://www.pishop.us/product/micro-servo-sg92r/)
- [Pololu 50:1 12V Brushed DC Motor w/ encoder](https://www.pololu.com/product/4753)
- [H-Bridge Motor Driver](https://www.pishop.us/product/hw-095a-l298-stepper-motor-driver-module-dc-dual-h-bridge/)
- [Raspberry Pi Pico H](https://www.pishop.us/product/raspberry-pi-pico-h-pre-soldered-headers/)
- [Raspberry Pi 4 Model B/4GB](https://www.pishop.us/product/raspberry-pi-4-model-b-4gb/)
- [DC 12V to 5V Buck Converter](https://www.pishop.us/product/dc-dc-12v-to-3-3v-5v-12v-power-module-multi-output-voltage-conversion/)
- [RPLIDAR A1](https://www.adafruit.com/product/4010)
- [Adafruit 9-DOF Absolute Orientation IMU](https://www.pishop.us/product/adafruit-9-dof-absolute-orientation-imu-fusion-breakout-bno055/)
- Some Camera (TBD, USB or PiCam)
- Some 3s LiPo (TBD, unknown MaH and amps)
- [Red 5mm LED (25 pack)](https://www.pishop.us/product/super-bright-red-5mm-led-25-pack/)









Useful links:
- [Calculate Amps of LiPo](https://www.rogershobbycenter.com/lipoguide/)
- [Medium article for OpenCV Camera publisher](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329)
- [NAV2 docs for camera calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)


Camera Detection Software
- [ROS2 v4l2 camera driver](https://github.com/tier4/ros2_v4l2_camera)
- [ROS2 image pipeline](https://github.com/ros-perception/image_pipeline/tree/humble)
- [ROS2 image common](https://github.com/ros-perception/image_common/tree/humble)
- [ROS2 shared (needed?)](https://github.com/ptrmu/ros2_shared)


Getting the PiCam to work with Ubuntu
1. Add `start_x=1` to `/boot/firmware/config.txt` 
2. reboot
3. in python, use `cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)`
    Optionally:
    - cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    - cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)