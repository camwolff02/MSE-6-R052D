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

Getting ROS2 set up
- [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Configuring Environment (do 1 and 2)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
- [Using colcon to build packages (Install colcon, Setup colcon_cd, Setup colcon tab completion](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#)

Getting micro-ROS set up
- [Install micro-ROS Raspberry Pi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk/tree/humble)
      - To find the Raspberry Pi Pico, use `sudo fdisk -l`. The pico, listed as Disk model: RP2, should be towards the bottom. Note the Disk, for example mine is `Disk /dev/sda`
      - In `/micro_ros_raspberrypi_pico_sdk/build`, instead of running the tutorial's copy command, run `sudo cp pico_micro_ros_example.utf /dev/sda` (replace with your pico directory)
- [Build micro-ROS setup](https://github.com/micro-ROS/micro_ros_setup/tree/humble#building)
- After building micro ros, build micro ros agent by running the following commands:
      - `ros2 run micro_ros_setup create_agent_ws.sh`
      - `ros2 run micro_ros_setup build_agent.sh`
      - '  

Getting the PiCam to work with Ubuntu
1. run `sudo apt update && sudo apt-get install python3-opencv`
2. (as sudo) In `/etc/needrestart/needrestart.conf`, uncomment `#$nrconf{ucodehints} = 0;`
3. (as sudo) In `/boot/firmware/config.txt`, append `start_x=1`
4. run `sudo apt install raspi-config`
5. run `sudo raspi-config`
      - select `3 Interface Options`
      - Enable legacy camera
      - Finish
6. reboot `sudo shutdown -r now`
7. in python, use `cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)`
    Optionally:
    - `cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)`
    - `cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)`
