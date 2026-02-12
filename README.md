# RAPTOR-XTDrone

RAPTOR-XTDrone 是在 XTDrone（PX4 + ROS + Gazebo）基础上整理的统一工作区，目标是后续集成 RAPTOR 端到端算法并在仿真中验证。

## 1. 平台概览（基于 XTDrone 论文）

XTDrone 是一个模块化仿真平台，核心链路为：
- PX4 SITL：飞控与状态估计
- Gazebo：物理与传感器仿真
- ROS/MAVROS：算法接口与通信

该平台支持多机协同、视觉/SLAM/规划等算法验证，并可将仿真流程迁移到实机。

## 2. 当前仓库结构

```text
RAPTOR-XTDrone/
├── XTDrone/        # XTDrone 功能包与脚本
├── PX4_Firmware/   # PX4 1.13 仿真相关代码
├── catkin_ws/      # ROS 工作空间（含 gazebo_ros_pkgs）
└── docs/paper/     # 参考论文
```

说明：本仓库已清理嵌套 Git 元数据（`.git/.gitmodules`），并通过根目录 `.gitignore` 忽略编译产物。

## 3. 依赖安装（Ubuntu 20.04 + ROS Noetic 推荐）

```bash
sudo apt update
sudo apt install -y ninja-build exiftool protobuf-compiler libeigen3-dev genromfs xmlstarlet \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3-pip gawk \
  python3-catkin-tools
pip3 install packaging numpy empy toml pyyaml jinja2 pyargparse kconfiglib jsonschema future
```

如遇依赖冲突，可尝试 `aptitude` 辅助解依赖。

## 4. 编译步骤

### 4.1 编译 catkin 工作空间（Gazebo ROS 插件）

```bash
cd ~/RAPTOR-XTDrone/catkin_ws
catkin_make
```

### 4.2 编译 PX4 SITL（PX4 1.13）

```bash
cd ~/RAPTOR-XTDrone/PX4_Firmware
DONT_RUN=1 make px4_sitl_default gazebo
```

## 5. `.bashrc` 环境激活（路径可切换）

将下面内容加入 `~/.bashrc`。只需改第一行路径变量：

```bash
# 方案A：本地目录是 ~/raptor-xtdrone
export RAPTOR_XTDRONE_ROOT=~/raptor-xtdrone
# 方案B：从 GitHub 克隆后目录是 ~/RAPTOR-XTDrone
# export RAPTOR_XTDRONE_ROOT=~/RAPTOR-XTDrone

source /opt/ros/noetic/setup.bash

if [ -f "${RAPTOR_XTDRONE_ROOT}/catkin_ws/devel/setup.bash" ]; then
  source "${RAPTOR_XTDRONE_ROOT}/catkin_ws/devel/setup.bash"
fi

source "${RAPTOR_XTDRONE_ROOT}/PX4_Firmware/Tools/setup_gazebo.bash" \
  "${RAPTOR_XTDRONE_ROOT}/PX4_Firmware" \
  "${RAPTOR_XTDRONE_ROOT}/PX4_Firmware/build/px4_sitl_default"

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${RAPTOR_XTDRONE_ROOT}/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${RAPTOR_XTDRONE_ROOT}/PX4_Firmware/Tools/sitl_gazebo
```

生效：

```bash
source ~/.bashrc
```

## 6. 基础运行验证

```bash
roslaunch px4 outdoor1.launch
```

新终端（等待 Gazebo 完全启动）：

```bash
python ~/RAPTOR-XTDrone/XTDrone/communication/multirotor_communication.py iris 0
python ~/RAPTOR-XTDrone/XTDrone/control/keyboard/multirotor_keyboard_control.py iris 0
```

若 Gazebo 进程未正常退出，可执行：

```bash
killall -9 gzclient
killall -9 gzserver
```

## 7. 首次上传到你的 GitHub

```bash
cd ~/RAPTOR-XTDrone
git init
git add .
git commit -m "chore: initial import of XTDrone PX4 workspace"
git branch -M main
git remote add origin git@github.com:Natsuago/RAPTOR-XTDrone.git
git push -u origin main
```

后续会在此仓库继续加入 RAPTOR 相关代码与实验脚本。
