#! /bin/bash

launch_cmd=$*

source /opt/cityfly/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/cityfly/share/fastrtps.xml

$launch_cmd
