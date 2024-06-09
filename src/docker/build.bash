#! /bin/bash

build_dev=1
build_run=0

# 遍历所有输入参数
for arg in "$@"; do
  case $arg in
    --dev)
      echo "Build image for dev"
      build_dev=1
      build_run=0
      ;;
    --run)
      echo "Build image for deploy, this requires build for dev first"
      build_dev=0
      build_run=1
      ;;
    --all)
      echo "Build image for dev and run"
      build_dev=1
      build_run=1
      ;;
    *)
      echo "Falling default, build image for dev"
      ;;
  esac
done

script_dir=$(cd $(dirname $0) && pwd)
workspace_dir=$(dirname $(dirname $script_dir))

if [ $build_dev -eq 1 ]; then
  echo "docker build -f $script_dir/Dockerfile.dev -t cityfly:dev $workspace_dir"
  docker build -f $script_dir/Dockerfile.dev -t cityfly:dev $workspace_dir
fi

if [ $build_run -eq 1 ]; then
  echo "docker build -f $script_dir/Dockerfile.run -t cityfly:run $workspace_dir"
  docker build -f $script_dir/Dockerfile.run -t cityfly:run $workspace_dir
fi
