#! /bin/bash

launch_cmd=$1

source /opt/cityfly/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/cityfly/share/fastrtps.xml

# ros2 launch $launch_cmd
ros2 run sensor oak_ffc_4p
