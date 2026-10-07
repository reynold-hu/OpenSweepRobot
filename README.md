<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="assets/logo-dark.png">
    <img src="assets/logo-light.png" alt="OpenSweepRobot" width="440">
  </picture>
</p>

<p align="center"><strong>An open-source robot vacuum you build yourself.</strong><br>
<sub>中文名 · 扫地僧</sub></p>

<p align="center">
  <img alt="status: pre-alpha" src="https://img.shields.io/badge/status-pre--alpha-orange">
  <img alt="license: Apache-2.0" src="https://img.shields.io/badge/license-Apache--2.0-blue">
  <img alt="ROS 2 Jazzy" src="https://img.shields.io/badge/ROS%202-Jazzy-22314E">
</p>

---

OpenSweepRobot is a robot vacuum you assemble and program yourself: 3D-printed
chassis, off-the-shelf and salvaged parts, ROS 2 on a Raspberry Pi, and a 2D LiDAR
for mapping and localization. It runs entirely on your local network. No cloud,
no account, no vendor app.

**Status: pre-alpha.** The project was started in October 2026. Nothing here
works end to end yet. See [Status](#status) for what actually exists.

## Why

Commercial robot vacuums are cheap, capable, and closed. You cannot change how
they clean, you cannot see what they know, and most of them route your floor plan
through someone else's server. Building one yourself is more expensive than buying
one — that is not the point. The point is having a machine you can fully inspect,
modify, and repair.

## Status

| Phase | What | State |
| :-- | :-- | :-- |
| 0 · Environment | ROS 2 + Gazebo + Nav2, fixed test world | Scene and scoring tools done; robot spawning pending |
| 1 · Baseline | Stock Nav2 + Regulated Pure Pursuit, measured | Not started |
| 2 · Contact tolerance | Disable collision prediction, add bumper peel-off reflex | Not started |
| 3 · Costmap | Calibrate inscribed radius and inflation for our body | Not started |
| 4 · Edge cleaning | LiDAR contour follower with cascaded steering | Not started |
| 5 · Hardware | Chassis, drive train, suction, cliff sensing | Not started |
| 6 · Docking | IR-beacon homing and charge contacts | Not started |

Phase 1 comes first on purpose. Without baseline numbers, no later comparison means
anything. The first milestone is a reproducible coverage percentage measured on a
fixed scene.

## Hardware

Modular, and every part is meant to be replaceable with a commodity equivalent.

| Part | Approach |
| :-- | :-- |
| Compute | Raspberry Pi 5 (4 GB) or CM4/CM5 running ROS 2 |
| Real-time MCU | STM32 — motors, sensors, charging, safety |
| Perception | 2D rotating LiDAR |
| Drive | Two differential wheels with encoders and suspension, plus a caster |
| Chassis | 3D printed |
| Power | 14.4 V Li-ion pack with in-pack BMS |

A large share of the mechanical parts come from salvaged Roborock and iRobot
units. Wear items on those machines are commodity sizes with deep aftermarket
supply, so replacements stay cheap and available.

## Software

| Layer | Choice |
| :-- | :-- |
| Framework | ROS 2 Jazzy |
| Navigation | Nav2 |
| Localization | `slam_toolbox` in localization mode, with wheel-encoder and IMU odometry fused through an EKF |
| Coverage | Planned. Nav2's own coverage server targets outdoor field geometry and does not accept a SLAM-built indoor map |
| Simulation | Gazebo |

We deliberately do not use visual SLAM. A vacuum sees the floor from a few
centimetres up, in low light, against low-texture flooring and blank walls —
precisely the conditions under which feature-based visual odometry degrades.

## Getting started

There is nothing to install yet. The first piece that exists is the simulation
benchmark: a fixed two-room scene and the tools that score a run against it.

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/sim/worlds/coverage_test.world
python3 sim/tools/check_scene.py sim/worlds/coverage_test.world
```

See [`sim/README.md`](sim/README.md) for the scene inventory and the measurement
protocol. Robot spawning is not wired up yet.

## Naming

The repository is `OpenSweepRobot` so that it searches well. The Chinese name is
**扫地僧**, after the sweeping monk of Shaolin — the one who looks like a janitor
and turns out to be the strongest person in the building.

## Prior art and acknowledgements

This project stands on other people's work. Specifically:

- **[OOMWOO](https://github.com/makerspet/oomwoo)** by Maker's Pet — an open-source
  robot vacuum with an open hardware, firmware and ROS 2 stack. Our architecture
  and bill of materials follow theirs closely. Apache-2.0.
- **[Nav2](https://github.com/ros-navigation/navigation2)** and
  **[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)** — navigation and
  localization.
- **[Valetudo](https://github.com/Hypfer/Valetudo)** — the reference point for what
  local-only vacuum control should look like.
- Many of our parts are salvaged from **Roborock** and **iRobot** machines; those
  companies' hardware is not affiliated with this project.

## License

Apache-2.0. See [LICENSE](LICENSE).

---

## 中文说明

**OpenSweepRobot（扫地僧）** 是一台自己组装、自己编程的开源扫地机器人：3D 打印底盘、
市售与拆机件、树莓派上跑 ROS 2、2D 激光雷达做建图与定位。完全本地运行，不接云、不需要账号。

**当前状态：pre-alpha，2026 年 10 月立项，尚无任何可以端到端跑通的环节。**

我们**不用视觉 SLAM**。扫地机在离地几厘米的高度工作，弱光、低纹理地面、纯白墙——
正好是特征法视觉里程计最容易失效的工况。定位方案是 `slam_toolbox` 定位模式，
配合轮式编码器与 IMU 的 EKF 融合。

第一阶段先做仿真里的覆盖率基线：没有基线数字，后面所有优化对比都没有意义。

架构与物料清单参考 **[OOMWOO](https://github.com/makerspet/oomwoo)**，机械件大量复用
石头与 iRobot 的拆机件——那些磨损件都是通用尺寸，供应充足、便宜好换。

许可证 Apache-2.0。
