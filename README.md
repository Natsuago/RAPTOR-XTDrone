# RAPTOR-XTDrone

## 平台概览
本仓库将 XTDrone 仿真基础（PX4 + Gazebo Classic + ROS）与 RAPTOR 外部模块集成到同一工作区，用于后续端到端策略验证与上板迁移。

本次为首次集成版本：RAPTOR 模块已编入 PX4 SITL 二进制，并补齐了 XTDrone(PX4 v1.13) 的兼容适配。

适配细节与后续增量更新统一维护在：[`docs/xtdrone-raptor-adaptation.md`](docs/xtdrone-raptor-adaptation.md)

## 项目结构
- `PX4_Firmware/`: XTDrone 使用的 PX4 固件源码（已接入 RAPTOR 外部模块编译）
- `raptor_external_modules/`: RAPTOR 相关 PX4 外部模块（policy/commander/multiplexer）
- `XTDrone/`: XTDrone 的 ROS 功能包、launch、通信与控制脚本
- `catkin_ws/`: ROS 工作空间
- `docs/`: 论文与适配文档

## 编译步骤
1. 编译 ROS 工作空间
```bash
cd ~/raptor-xtdrone/catkin_ws
catkin_make
```

2. 准备 RLtools 根目录（用于 RAPTOR 模块头文件）
```bash
git clone https://github.com/rl-tools/rl-tools.git ~/rl-tools
```

3. 编译 PX4 SITL（含 RAPTOR 外部模块）
```bash
cd ~/raptor-xtdrone/PX4_Firmware
PX4_ALLOW_NO_GIT=1 GIT_SUBMODULES_ARE_EVIL=1 make px4_sitl_default \
  EXTERNAL_MODULES_LOCATION=~/raptor-xtdrone/raptor_external_modules \
  RL_TOOLS_ROOT=~/rl-tools -j8
```

## 环境激活
将以下内容加入 `~/.bashrc`（按本地目录二选一）：
```bash
# export RAPTOR_XTDRONE_ROOT=~/raptor-xtdrone
# export RAPTOR_XTDRONE_ROOT=~/RAPTOR-XTDrone

source /opt/ros/noetic/setup.bash
source ${RAPTOR_XTDRONE_ROOT}/catkin_ws/devel/setup.bash
source ${RAPTOR_XTDRONE_ROOT}/PX4_Firmware/Tools/setup_gazebo.bash \
  ${RAPTOR_XTDRONE_ROOT}/PX4_Firmware \
  ${RAPTOR_XTDRONE_ROOT}/PX4_Firmware/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${RAPTOR_XTDRONE_ROOT}/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${RAPTOR_XTDRONE_ROOT}/PX4_Firmware/Tools/sitl_gazebo
```
