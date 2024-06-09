#! /bin/sh

workspace=$1

docker run -it \
  --privileged \
  --ipc host \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $workspace:/home/ubuntu/cityfly \
  cityfly:dev
