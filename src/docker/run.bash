#! /bin/sh

launch_cmd=$*

docker run -it \
  --privileged \
  --ipc host \
  --network host \
  --volume /dev:/dev \
  cityfly:run \
  $launch_cmd
