# Dexterous Hand Sim-to-Real Controller
(English | [中文版](#-中文介绍))

A C++ based Sim-to-Real motion control framework for dexterous hands (tested on 16-DOF Allegro Hand and 6-DOF Inspire-Robots RH56F1). The project implements kinematics, impedance control, and hardware communication from scratch without relying on external IK solvers.

> **Demo Video:** [Link to your Sim-to-Real Pinching Video](https://b23.tv/PvVZ5lQ)

## Core Features
- **Native Kinematics Implementation**: Explicit SE(3) homogeneous transformation chain and geometric Jacobian derivation `(T.linear() * axis).cross(P_tip - P)`.
- **Singularity Handling**: Applied Damped Least Squares (DLS) pseudo-inverse $J^T(JJ^T + \lambda^2 I)^{-1}$ to ensure numerical stability near workspace boundaries.
- **Impedance Control & Integrator**: Integrated a virtual spring model with a command integrator (`cmd += q_dot * dt`) to mitigate static friction and steady-state errors on real hardware.
- **Hardware Communication Bridge**: Implemented a byte-stream sliding window parser for half-duplex RS485 communication, using `std::mutex` to resolve data race conditions between ROS 2 async callbacks and the control loop.

## Node Architecture
The system is decoupled into two primary ROS 2 nodes (Producer-Consumer pattern):

### 1. Controller Node: `thumb_index_catch`
- Runs on a strict 50Hz (20ms) Timer Callback.
- Computes Forward Kinematics (FK) and analytical Jacobian.
- Generates a collision-free pre-grasp trajectory using Smooth-Step (Cosine) interpolation.
- Calculates the DLS pseudo-inverse and outputs the integrated joint commands.

### 2. Hardware Bridge Node: `inspire_bridge_node`
- Manages RS485 serial communication with the Inspire-Robots RH56F1.
- Uses a sliding window state machine to capture headers (`0x90 0xEB`), reducing frame parsing errors under non-real-time Linux environments.
- Utilizes `std::mutex` to decouple the asynchronous ROS 2 subscription from the hardware control loop.

---

## 中文介绍

本项目是一个面向灵巧手（Allegro Hand & 因时 RH56F1）的 Sim-to-Real 运动控制代码库。核心运动学算法与底层通信驱动均由 C++ 编写，未调用外部 IK 求解器库。

> **实机演示视频：** [点击观看：灵巧手 Sim-to-Real 闭环对捏实测](https://b23.tv/PvVZ5lQ)

### 核心技术实现
- **正运动学与雅可比**：基于 SE(3) 齐次变换矩阵推导连杆正运动学，并使用一阶向量叉乘计算全局解析几何雅可比矩阵 `(T.linear() * axis).cross(P_tip - P)`。
- **奇异点处理**：引入阻尼最小二乘法（DLS），计算矩阵伪逆 $J^T(JJ^T + \lambda^2 I)^{-1}$，保证算法在工作空间边界时的数值稳定性。
- **控制策略设计**：采用虚拟弹簧阻抗控制模型，结合纯指令积分器 (`cmd += q_dot * dt`) 减小真机静摩擦带来的稳态误差；引入余弦平滑插值（Smooth-Step）规划防干涉轨迹。
- **底层通信处理**：针对因时灵巧手的 RS485 半双工串口，编写了基于字节流滑动窗口的解析算法以应对粘包错位；并使用 `std::mutex` 解决 ROS 2 异步回调与定时器控制循环之间的数据竞态问题。

### 节点架构
系统采用“算法控制”与“底层通信”解耦的设计：

#### 1. 控制器节点：`thumb_index_catch`
- 运行于 50Hz (20ms) 的定时器线程。
- 负责运动学推导、雅可比计算以及 DLS 正则化求逆。
- 输出积分限幅后的期望关节角至 ROS 2 话题。

#### 2. 底层驱动桥：`inspire_bridge_node`
- 负责非实时 Linux 系统下微电缸串口的稳定收发。
- 动态寻址帧头，过滤硬件抖动导致的错位断帧。
- 内置物理电缸行程到关节弧度 (Rad) 的线性映射与硬限位保护。

### 编译运行
- **环境**: Ubuntu 22.04 / ROS 2 (Humble) / C++ 17 / Eigen 3
- **编译**: `colcon build --packages-select allegro_commander`
