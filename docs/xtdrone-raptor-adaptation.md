# XTDrone-RAPTOR 适配说明

## 1. 版本基线
- XTDrone 论文发布时间：2020-03（平台较早，主链路基于 Gazebo Classic）
- 当前 PX4 基线：XTDrone 的 PX4 v1.13 系列（本仓库对应 `PX4_Firmware` 当前代码）
- RAPTOR 论文提交时间：2025-09（算法与工程依赖更新）
- RAPTOR 集成来源：`rl-tools/px4` 外部模块方案（首次集成到本仓库）

## 2. 首次集成已完成适配
### 2.1 工程与构建适配
- 将 RAPTOR 外部模块引入到 `raptor_external_modules/`
- 将 `policy.h` 策略 blob 接入 `rl_tools_policy`
- 适配“去子仓库化”工作区：
  - 允许 PX4 在父级 Git 仓库中构建
  - 子模块检查改为可跳过/按存在性处理
  - 缺失 `flightgear_bridge` 时跳过对应外部工程构建

### 2.2 PX4 v1.13 接口适配
- 补充 `actuator_motors` 话题别名：`actuator_motors_rl_tools`、`actuator_motors_mux`
- 适配 uORB 头文件/消息结构体命名差异（`RlTools*`）
- 适配 `manual_control_setpoint` 字段差异（v1.13 无 `buttons`，改为 `aux1..aux6` 映射）
- 暂时关闭 `rl_tools_benchmark`（当前组合下接口不兼容，不影响 policy 主链路）

### 2.3 链接与版本信息适配
- 在 `version.c` 增加 `px4_mavlink_lib_version_binary()` 回退实现，解决无嵌套 mavlink git 元数据时的链接问题

### 2.4 XTDrone legacy 输出链接管适配（SITL）
- 目标：不改 XTDrone 默认主链路前提下，让 RAPTOR 真正接管输出。
- 实现位置：`PX4_Firmware/src/lib/mixer_module/mixer_module.cpp`（static mixer 分支）。
- 实现方式：
  - 订阅 `rl_tools_policy_status` 与 `actuator_motors_rl_tools`；
  - 仅采信 `exit_reason==NONE` 的策略状态并做短时保持，避免 400Hz 下状态抖动导致频繁回切；
  - 将 RAPTOR 电机输出（`actuator_motors_rl_tools`）按 `quad_wide` 归一化混控矩阵逆变换为 `actuator_controls_0` 的 `roll/pitch/yaw/throttle`；
  - 继续复用 XTDrone 原链路：`actuator_controls_0 -> quad_w mixer -> pwm_out_sim`。
- 回退策略：
  - 若策略失活、状态超时或电机话题超时，则自动回退到 PX4 默认控制链。
- 运行验证（SITL）：
  - 策略运行时 `actuator_outputs[0..3]` 可直接跟随 `actuator_motors_rl_tools`；
  - 停止 `rl_tools_policy` 后，输出自动回落到默认链路的低油门状态。

## 3. SITL 频率对齐（为后续 HITL 准备）
官方 RAPTOR 建议频率对齐到高频（常见 400Hz）。

结合 XTDrone 的 Gazebo Classic 代码，`enable_lockstep=1` 时 `real_time_update_rate` 被限制为 250 的整数倍；为避免破坏 XTDrone 默认链路，本仓库采用“独立 RAPTOR 配置”方式：

- 新增 world：`PX4_Firmware/Tools/sitl_gazebo/worlds/empty_raptor_400.world`
  - `max_step_size=0.0025`
  - `real_time_update_rate=400`
- 新增模型：`PX4_Firmware/Tools/sitl_gazebo/models/iris_raptor/iris_raptor.sdf`
  - `mavlink_interface.enable_lockstep=0`（仅 RAPTOR 专用模型）
- 新增启动文件：`XTDrone/sitl_config/launch/raptor_sitl_iris.launch`
  - 使用 `iris` 飞控模型 + `iris_raptor` SDF + `empty_raptor_400.world`

> 说明：该方案不改 XTDrone 现有默认 launch，避免影响原教程流程。

## 4. 推荐参数（SITL/HITL共用检查清单）
- `IMU_GYRO_RATEMAX=400`
- `IMU_GYRO_CUTOFF>=100`
- `COM_DISARM_LAND=-1`
- `COM_DISARM_PRFLT=-1`
- `MC_*_I=0`（策略接管期间避免积分累积）

如用视觉定位（mocap/VIO），再按任务配置 `EKF2_HGT_REF`、`EKF2_BARO_CTRL`、`EKF2_MAG_TYPE` 等参数。

## 5. 变更记录（持续追加）
- 2026-02-12：首次完成 RAPTOR 外部模块集成、PX4 v1.13 兼容修复、RAPTOR 专用 400Hz SITL 配置。
- 2026-02-12：完成 XTDrone legacy 输出链接管适配（static mixer 内注入 RAPTOR 控制），并验证策略停更时可自动回退默认链路。

后续每次适配更新请在本文件追加记录，不在 README 中堆叠细节。
