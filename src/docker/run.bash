#! /bin/bash

launch_cmd=$*

docker run -it \
  --rm \
  --privileged \
  --ipc host \
  --network host \
  --volume /dev:/dev \
  --volume `pwd`:/workspace \
  cityfly:run \
  $launch_cmd
