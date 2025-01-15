#! /bin/bash

cd $(dirname $0)
cd ..

source install/setup.bash
ros2 launch tmp/auto_aim_launch.py
