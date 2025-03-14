#!/bin/bash

cd $(dirname $0)

source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
