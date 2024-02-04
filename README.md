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
- [Raspberry Pi Camera Module 3 Wide](https://www.pishop.us/product/raspberry-pi-camera-module-3-wide/)
- [Red 5mm LED (25 pack)](https://www.pishop.us/product/super-bright-red-5mm-led-25-pack/)
- - Some 3s LiPo (TBD, unknown MaH and amps)






## Building From Source 
      
note that with the exception of section 2. Getting ROS2 set up, you shouldn't have to click on any of the links. You should just be able to run the commands, and refer to the links if you're having any difficulties or problems

--------------------------------------------------------------------------------------------------------------------------------------------
1. Clone the repo
      - `git clone --recurse-submodules https://github.com/camwolff02/MSE-6-R052D.git`
      - `cd MSE-6-R052D`
   
--------------------------------------------------------------------------------------------------------------------------------------------
2. Getting ROS2 set up
      - [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
      - [Configuring Environment (do 1 and 2)](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
      - [Using colcon to build packages (Install colcon, Setup colcon_cd, Setup colcon tab completion](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#)

--------------------------------------------------------------------------------------------------------------------------------------------
3. Getting micro-ROS set up (start each step at root repo directory)
      - [Setup Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
        - `cd lib/pico_sdk`
        - `mkdir build && cd build`
        - `cmake ..`
        - `make`
        - add environment variable to startup script `echo "export PICO_SDK_PATH=$(pwd)" >> ~/.bashrc`
        - source the shell startup script `source ~./.bashrc`

      - [Setting up Picotool](https://github.com/raspberrypi/picotool)
        - `cd lib/picotool`
        - `mkdir build && cd build`
        - `cmake ..`
        - `make`

      - [Build the Micro-ROS Raspberry Pi Pico Firmware](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)
        - `cd lib/micro_ros_raspberrypi_pico_sdk`
        - `mkdir build && cd build`
        - `cmake ..`
        - `make`
      
      - Flash the Micro-ROS example firmware
        - `cd lib/picotool/build'` 
        - `sudo ./picotool load /home/pi/MSE-6-R052D/lib/micro_ros_raspberrypi_pico_sdk/build/pico_micro_ros_example.uf2 -f`
      
      - [Build the micro-ROS Agent](https://github.com/micro-ROS/micro_ros_setup/tree/humble#building)
        - `cd uros_ws`
        - `rosdep update && rosdep install --from-paths src --ignore-src -y`
        - `colcon build`
        - `source install/local_setup.bash`
        - `ros2 run micro_ros_setup create_agent_ws.sh`
        - `ros2 run micro_ros_setup build_agent.sh`

      - Run the MicroROS Bridge
        - `cd uros_ws`
        - `. install/setup.bash`
        - `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
   
      - Verify micro-ROS is working
        - In a separate terminal, `ros2 topic list`
        - If you see /pico_publisher, run `ros2 topic echo pico_publisher`
        - If the pico starts up, the micro-ROS agent on your computer is communicating with the pico! otherwise, something went wrong in setup

      - Rebuilding Micro-ros with our Servo message
            - `cd uros_ws`
            - build the servo package `colcon build --packages-select ros2_servo`
            - source built package `source install/local_setup.bash`
            - Crate firmware workspace and generate library with `ros2 run micro_ros_setup create_firmware_ws.sh generate_lib`
            - run ros2 servo workspace create function to copy source code for our message into the firmware workspace `ros2 run ros2_servo create_fwws.sh`
            - start the firmware build `ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_toolchain.cmake $(pwd)/my_colcon.meta`
      

--------------------------------------------------------------------------------------------------------------------------------------------
< TODO TEST TO MAKE SURE THIS IS REPEATABLE >

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
5. Calibrating your camera
      - [Install ros2_v4l2_camera](https://github.com/tier4/ros2_v4l2_camera) `sudo apt install ros-humble-v4l2-camera`
      - Install image transport for image compression `sudo apt install ros-humble-image-transport-plugins`
      - Start publishing images with `ros2 run v4l2_camera v4l2_camera_node`
      - test with `ros2 run rqt_image_view rqt_image_view` (preferred) or `ros2 run image_view image_view --ros-args --remap /image:=/image_raw`
      - Run calibrator `ros2 run camera_calibration cameracalibrator \--size=8x6 \ --square=0.063 \ --approximate=0.3 \ --no-service-check \ --ros-args --remap /image:=/image_raw`
        - Make sure to change the parameters `size` and `square` to be accurate to the checkerboard your're using
        - Get your camera, and move around the checkerboard until the "calibrate" button lights up. Press it
        - Once calibration is finished and the calibration text is output to the terminal, hit the "save" button, and exit the program 
      - create calibration folder and move there`mkdir calib/my_camera_name && cd calib/my_camera_name`
      - move calibration files and extract them `mv /tmp/calibrationdata.tar.gz . && tar -xf calibrationdata.tar.gz`
   
--------------------------------------------------------------------------------------------------------------------------------------------
6. [Build and run ArUco package](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/opencv_4.7)
      - install dependency `pip3 install opencv-contrib-python transforms3d && sudo apt install ros-humble-tf-transformations`
      - `cd ros2_ws`
      - `colcon build --packages-select ros2_aruco ros2_aruco_interfaces`
      - `source install/setup.bash`
      - `ros2 run ros2_aruco aruco_node --ros-args -p marker_size:=0.145 -p aruco_dictionary_id:=DICT_4X4_50 -p image_topic:=/image_raw -p camera_info_topic:=/camera_info -p camera_frame:=map` (note that we're specifying the camera frame to be the map, that means positions are given relative to the map. In the final software stack, this should be changed to the frame of the camera) 
      - now we can test by running `ros2 topic list`. If you see `/aruco_markers`, detection is working!
      - We can see this in action by running `ros2 topic echo /aruco_markers` (make sure you're in `ros2_ws` and have sourced your environment so ROS can tell what the aruco message is). Notice how at first, the terminal pauses and nothing prints. But, hold up your marker(s), and you'll see them printed out with the correct id(s)!
      - We can even visualize our markers in 3D space! Try running `rviz2 -d rviz/aruco_demo.rviz` and watch as the red arrow moves around with your marker. That arrow is a normal vector to where ROS thinks your aruco tag is, relative to (0,0,0)
--------------------------------------------------------------------------------------------------------------------------------------------


Useful links:
- [Calculate Amps of LiPo](https://www.rogershobbycenter.com/lipoguide/)
- [Medium article for OpenCV Camera publisher](https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329)
- [NAV2 docs for camera calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [Micro ROS on Pico](https://www.youtube.com/playlist?list=PLspDyukWAtRU6CExohVFg07T98ssxdqy1)

