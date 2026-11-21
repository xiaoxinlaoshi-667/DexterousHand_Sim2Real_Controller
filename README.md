# Dexterous Hand Sim-to-Real Control Project
(English | [中文版](#-中文介绍))

This is a robotics practice project where I implemented motion controllers for both a simulated 16-DOF Allegro Hand and a physical 6-DOF Inspire-Robots RH56F1 hand. The project uses ROS 2 and C++ without relying on external IK libraries.

> **Demo Video:** [Link to your Sim-to-Real Pinching Video](https://b23.tv/PvVZ5lQ)

## Implementation Details
- **Kinematics & Jacobian**: Derived the SE(3) forward kinematics and geometric Jacobian manually using `Eigen`. Handled the specific mechanical linkage ratios (e.g., 0.8962, 1.1545) of the physical RH56F1 hand.
- **Control Strategy**: 
  - Used Damped Least Squares (DLS) pseudo-inverse to handle kinematic singularities.
  - Formulated a relative Jacobian (`J_rel << J_index, -J_thumb`) to control the relative distance between two fingers.
  - Added a command integrator (`target_q += q_dot * dt`) to overcome static friction on the real hardware.
- **Pre-Grasp Logic**: Added a simple two-phase state machine to move the thumb out of the way before closing the fingers, avoiding self-collision.
- **Serial Communication**: Wrote a basic RS485 serial reader using a sliding window to find data frame headers (`0x90 0xEB`), and used `std::mutex` to prevent data races between ROS 2 callbacks and the control loop.

## Code Structure
The workspace consists of three main C++ nodes:

- `thumb_index_catch.cpp`: **Simulation Controller**. Controls the 16-DOF Allegro Hand in Ignition Gazebo, calculating 3D kinematics and DLS inverse kinematics at 50Hz.
- `inspire_commander.cpp`: **Real-Hand Controller**. Controls the 6-DOF Inspire RH56F1 physical hand, handling the mechanical coupling and integral control to overcome real-world friction.
- `inspire_bridge_node.cpp`: **Hardware Driver**. Handles the actual RS485 byte-stream parsing and publishes ROS 2 topics.

---

## 🇨🇳 中文介绍

这是我在学习机器人控制算法时完成的一个实践项目。项目基于 ROS 2 和 C++，为 16 自由度的 Allegro Hand (仿真) 和 6 自由度的因时 RH56F1 (真机) 编写了从头推导的运动学闭环控制程序。

> **实机演示视频：** [点击观看：灵巧手闭环对捏实测](https://b23.tv/PvVZ5lQ)

### 主要工作内容
- **纯手写运动学**：使用 Eigen 库推导了 SE(3) 齐次矩阵和几何雅可比。针对因时实机，在代码中计算了内部连杆的耦合传动比（1.1545 / 0.8962）。
- **相对雅可比与防奇异点**：将双指的雅可比拼接为相对雅可比矩阵 (`J_rel << J_index, -J_thumb`) 来控制指尖距离误差，并加入了 DLS（阻尼最小二乘法）伪逆来处理奇异点。
- **抗摩擦与防碰撞设计**：
  - 为克服真实物理电缸的静摩擦，在控制率中加入了指令积分累加机制。
  - 写了一个简单的阶段状态机，在抓取前先让大拇指外旋避障，避免了真机两指发生碰撞干涉。
- **底层串口通信**：针对 RS485 通信，写了一个带滑动窗口的字节流解析函数来过滤错位数据，并用 `std::mutex` 解决了 ROS 2 异步回调和定时器之间的数据冲突。

### 节点说明
- `thumb_index_catch.cpp` (仿真算法节点)：负责 16 轴 Allegro Hand 的运动学推导与 50Hz 闭环控制。
- `inspire_commander.cpp` (实机算法节点)：负责 6 轴因时实机的运动学映射与积分器控制。
- `inspire_bridge_node.cpp` (硬件通信节点)：负责 Linux 环境下 RS485 串口数据的解析和收发。
