set up platformio udev rules: https://docs.platformio.org/en/latest/core/installation/udev-rules.html

run the micro-ros-agent:
    - sudo docker run -it --net=host --privileged microros/micro-ros-agent:humble serial --dev /dev/ttyACM0

run the controller node:
    - ros2 launch teleop_twist_joy teleop-launch.py config_filepath:='ps4.yml'