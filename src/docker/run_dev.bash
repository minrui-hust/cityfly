#! /bin/bash

script_dir=$(cd $(dirname $0) && pwd)
workspace_dir=$(dirname $(dirname $script_dir))

docker run -it \
  --privileged \
  --ipc host \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $HOME/.ssh:/home/ubuntu/.ssh \
  --volume $workspace_dir:/home/ubuntu/cityfly \
  cityfly:dev
