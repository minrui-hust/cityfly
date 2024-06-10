# Introduction
cityfly(低空飞行)，是一个基于ros2(jazzy)的用于无人机低空飞行的软件算法栈，它包含了sensor驱动，融合定位，融合感知，避障规划，运动控制，诊断等模块。其目标是完成无人机低空的指哪飞哪，操作者只需要关心无人机要去哪，剩下的交给cityfly。

# Setup
为了省去繁杂的开发环境配置过程，cityfly的开发和部署均基于docker，支持使用tmux+vim的组合以及vscode remote的方式进行开发。setup开发环境的主要包含一下步骤：
1. 安装docker
2. 编译dev镜像
3. 进入容器

**安装docker**
docker的安装参考官方教程，(Install Docker Engine on Ubuntu)[https://docs.docker.com/engine/install/ubuntu/],记得完成(Post-installation steps for Linux)[https://docs.docker.com/engine/install/linux-postinstall/]

**编译dev镜像**
首先clone仓库到本地(cityfly的仓库是一个标准的ros wrokspace):
```bash
git clone https://github.com/minrui-hust/cityfly.git
cd cityfly
```
编译dev镜像：
```bash
./src/docker/build.bash --dev
```
其中--dev用于指定编译开发用的dev镜像，另外还可以是--run(部署用的镜像)，--all(dev和run)。  
编译完成后使用docker images 查看生成的镜像，其tag应该为cityfly:dev
```bash
docker images
```

