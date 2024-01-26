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

--------------------------------------------------------------------------------------------------------------------------------------------

Building From Source
1. Clone the repo
      - `git clone --recurse-submodules https://github.com/camwolff02/MSE-6-R052D.git`
      - `cd MSE-6-R052D`

2. Getting ROS2 set up
      - [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
      - [Configuring Environment (do 1 and 2)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
      - [Using colcon to build packages (Install colcon, Setup colcon_cd, Setup colcon tab completion](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#)

3. Getting micro-ROS set up (start each step at root repo directory)
      - [Build the micro-ROS Agent](https://github.com/micro-ROS/micro_ros_setup/tree/humble#building)
        - `cd uros_agent_ws`
        - `rosdep update && rosdep install --from-paths src --ignore-src -y`
        - `colcon build`
        - `source install/local_setup.bash`
        - `ros2 run micro_ros_setup create_agent_ws.sh`
        - `ros2 run micro_ros_setup build_agent.sh`

      - [Build and flash the Micro-ROS Raspberry Pi Piko Firmware](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)
        - `cd micro_ros_raspberrypi_pico_sdk`
        - `mkdir build`
        - `cd build`
        - `cmake ..`
        - `make`
        - Hold down the BOOTSEL button, plug in your pico, and let go
        - To find the Raspberry Pi Pico, use `sudo fdisk -l`. The pico, listed as Disk model: RP2, should be towards the bottom. Note the Disk, for example mine is `Disk /dev/sda`
        - Copy the firmware to the pico with `sudo cp pico_micro_ros_example.uf2 /dev/sda` (replacing `/dev/sda` with your disk)
         
      - Run the MicroROS Bridge
        - `cd uros_agent_ws`
        - `. install/setup.bash`
        - `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
   
      - Verify micro-ROS is working
        - In a separate terminal, `ros2 topic list`
        - If you see /pico_publisher, run `ros2 topic echo pico_publisher`
        - If the pico starts up, the micro-ROS agent on your computer is communicating with the pico! otherwise, something went wrong in setup

<VERIFIED AND WORKING UP TO THIS POINT>

4. Getting the PiCam to work with Ubuntu
      - run `sudo apt update && sudo apt-get install python3-opencv`
      - (as sudo) In `/etc/needrestart/needrestart.conf`, uncomment `#$nrconf{ucodehints} = 0;`
      - (as sudo) In `/boot/firmware/config.txt`, append `start_x=1`
      - run `sudo apt install raspi-config`
      - run `sudo raspi-config`
        - select `3 Interface Options`
        - Enable legacy camera
        - Finish
      - reboot `sudo shutdown -r now`
      - To test, run `python3 test_camera.py`. If see `test_image.png` and the image looks good, the camera is working!
  
--------------------------------------------------------------------------------------------------------------------------------------------


Useful links:
- [Calculate Amps of LiPo](https://www.rogershobbycenter.com/lipoguide/)
- [Medium article for OpenCV Camera publisher](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329)
- [NAV2 docs for camera calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [Micro ROS on Pico](https://www.youtube.com/playlist?list=PLspDyukWAtRU6CExohVFg07T98ssxdqy1)


Camera Detection Software
- [ROS2 v4l2 camera driver](https://github.com/tier4/ros2_v4l2_camera)
- [ROS2 image pipeline](https://github.com/ros-perception/image_pipeline/tree/humble)
- [ROS2 image common](https://github.com/ros-perception/image_common/tree/humble)
- [ROS2 shared (needed?)](https://github.com/ptrmu/ros2_shared)

