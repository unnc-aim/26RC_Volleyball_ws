# 26_RC_Volleyball 排球机器人工作空间

本项目为 26_RC 排球机器人（Volleyball Robot）的 ROS 2 上位机与控制中枢工作空间，基于 **Ubuntu (ROS 2 Jazzy)** 开发，支持 EtherCAT 工业总线实时控制、Stewart 六自由度并联机构姿态解算、长基线双目视觉追踪以及高速击打时序规划。

---

## 一、 系统架构与功能包分布

```
26_RC_Volleyball_ws
├── src
│   ├── EcatV2_Master/           # [Submodule] EtherCAT 主站与从站通讯协议栈
│   ├── universal_controllers_v2/# [Submodule] 通用底盘/云台控制器与数学工具库
│   ├── soem_bringup/            # EtherCAT 节点启动配置与参数映射
│   ├── volleyball_controller/   # 排球机器人主控中枢 (VolleyballHub) 与 Stewart 逆解
│   ├── volleyball_vision/       # [Submodule] 迈德威视长基线双目视觉与 YOLO 空间测距
│   ├── volleyball_tracker/      # [Submodule] 二次方空气动力学 Aero-EKF 滤波与弹道落点预测
│   └── volleyball_planner/      # [Submodule] 底盘时优拦截速度规划与击打时序决策
```

---

## 二、 遥控器控制系统详解 (RC Control System)

排球机器人的遥控接收与决策中枢运行于 `volleyball_controller` 中的 `volleyball_hub_controller` 节点，以 **1ms (1000 Hz)** 的实时控制循环高频刷新。

### 1. 信号来源与通讯协议
- **订阅话题**：`/sbus_rc`
- **消息类型**：`custom_msgs/msg/ReadSBUSRC`
- **通道协议**：标准 S.BUS 协议 16 通道输入
- **基准标定**：中位基准值 `MID_VALUE = 992`，摇杆单向有效量程 `SCALE = 819`

### 2. 遥控器通道映射与功能分配表

| 通道 | 符号常量定义 | 物理按键 / 摇杆 | 控制功能说明 |
| :--- | :--- | :--- | :--- |
| **CH1** | `LEFT_X` (`channels[0]`) | 左摇杆水平轴 | **复用模式 A**：底盘自转速度 (`spin_speed`)<br/>**复用模式 B**：Stewart 靶面横滚角 (`roll`) |
| **CH2** | `LEFT_Y` (`channels[1]`) | 左摇杆垂直轴 | **复用模式 B**：Stewart 靶面俯仰角 (`pitch`) |
| **CH3** | `RIGHT_X` (`channels[2]`) | 右摇杆水平轴 | 底盘横向（左右 $Y$ 轴）全向平移速度 |
| **CH4** | `RIGHT_Y` (`channels[3]`) | 右摇杆垂直轴 | 底盘纵向（前后 $X$ 轴）全向平移速度 |
| **CH5** | `SWITCH_SERVE_2POS` (`channels[4]`) | 2 段拨杆开关 | **复合发球/挥拍击打时序**触发开关（> 1000 触发动作） |
| **CH6** | `SWITCH_ESTOP_BTN` (`channels[5]`) | 按键 / 拨片 | **急停开关**（> 992 触发系统级切断） |
| **CH7** | `SWITCH_Z_STRIKE` (`channels[6]`) | 2 段开关 | **单独顶球/上抛**动作触发开关（> 1000 触发） |
| **CH8** | `SWITCH_STEWART_TILT` (`channels[7]`) | 3 段开关 | 预留功能开关 |
| **CH9** | `channels[8]` | 开关/拨键 | 底盘强制锁止刹车（> 1000 强制线速度与角速度归 0） |
| **CH10**| `BACK_R` (`channels[9]`) | 遥控器背部拨键 | **左摇杆功能复用分流切换开关**（< 1000 底盘自转；> 1000 Stewart 靶面调姿） |

---

### 3. 安全联锁与急停保护 (Fail-Safe & E-Stop)

在每个 1ms 控制循环的入口，系统执行最高优先级硬件级联锁检测：
```cpp
if (!rc_received_ || !rc_.online || rc_.frame_lost || rc_.fail_safe 
    || rc_.channels[SWITCH_ESTOP_BTN] > RCConstants::MID_VALUE) {
    emergency_stop();
    return;
}
```

当发生以下任意异常时，即刻执行 `emergency_stop()`：
- 遥控器信号断联 (`online == false`)
- S.BUS 链路丢帧 (`frame_lost == true`) 或失控保护触发 (`fail_safe == true`)
- 操作手按下急停按钮 **CH6** (`channels[5] > 992`)

**急停执行动作**：
1. **底盘制动**：`emergency_stop = 1`，全向速度 $v_x = v_y = \omega = 0$，`extra_info[0] = 1`（机械锁止）。
2. **Stewart 平台复位**：`emergency_stop = 1`，姿态回平，高度恢复初始标称高度 $Z = 0.22186\text{ m}$。
3. **驱动电机下电**：LK 击打电机、DM 俯仰电机立即关闭使能（`enable = 0, torque = 0`）。
4. **板载气动/舵机断开**：PWM 4 个输出通道全部归零。

---

### 4. 底盘全向运动与 Stewart 姿态复用控制

#### (1) 底盘全向平移解算
右摇杆直接解算全向移动线速度（最大线速度 `MAX_CHASSIS_SPEED = 3.0 m/s`）：
- **前后移动**：$v_x = \frac{\text{RIGHT\_Y} - 992}{819} \times 3.0\text{ m/s}$
- **左右平移**：$v_y = -\frac{\text{RIGHT\_X} - 992}{819} \times 3.0\text{ m/s}$

#### (2) 背部开关 `BACK_R` 的左摇杆复用分流

```
                    ┌─── [BACK_R < 1000 (底盘旋转模式)] ───> 左摇杆水平控制底盘自转角速度 spin_speed
                    │                                         (Stewart 平台锁定水平 pitch=0, roll=0)
左摇杆 (LEFT_X/Y) ──┤
                    └─── [BACK_R > 1000 (靶面微调模式)] ───> 底盘自转锁定 (spin_speed = 0)
                                                             左摇杆垂直控制 Stewart 俯仰角 Pitch
                                                             左摇杆水平控制 Stewart 横滚角 Roll
```

- **底盘自转模式**（`BACK_R < 1000`）：
  $$\omega_{\text{spin}} = -\frac{\text{LEFT\_X} - 992}{819} \times 3.0\text{ rad/s}$$
- **Stewart 靶面调姿模式**（`BACK_R > 1000` 且发球处于完成状态）：
  $$\text{Pitch} = -\frac{\text{LEFT\_Y} - 992}{819} \times 0.16\text{ rad}$$
  $$\text{Roll} = -\frac{\text{LEFT\_X} - 992}{819} \times 0.12\text{ rad}$$
  此时位置保持中立（$X = 0, Y = 0, \text{Yaw} = 0$）。

---

### 5. 单独顶球/上抛控制 (Push 状态机)

当拨动开关 **CH7**（`channels[6] > 1000`）时，Stewart 平台执行垂直单次上抛顶球动作：

```
[WAITING] ──────(CH7 > 1000)──────> [GOING_UP] ──────(超时 0.18s)──────> [RESETTING] ──────(超时 0.15s)──────> [WAITING]
(Z = 0.20m)                         (Z = 0.36m 顶升推球)                   (Z = 0.20m 回落复位)                  (就绪待命)
```

> **抛球时底盘防抖联动**：在 Push 机构处于 `GOING_UP` 顶升过程或 **CH9**（`channels[8] > 1000`）激活时，底盘速度被强制清零并置 `extra_info[0] = 1` 锁止轮组，防止底盘晃动干扰抛球垂直度。

---

### 6. 复合发球/击打动作时序 (Strike 状态机)

发球与击打是排球机器人动作链中最关键的复合时序，通过开关 **CH5**（`channels[4] > 1000`）触发，经历 6 个阶段顺序推进：

```
  [1. WAITING]  ──(CH5 拨上)──>  [2. PUSHING]   ──(顶球复位)──>  [3. LAYING_DOWN]
  挥拍位待命                    顶球机构推升球体                 大Pitch倒下避让
        │                                                               │
  [6. DONE]     <──(编码器<55555) [5. GOING_UP] <──(编码器>50000) [4. GOING_DOWN]
  双环PID回位收拍               大扭矩击球顺势完成               大扭矩向下极速挥拍
```

1. **`WAITING`（待命准备）**：
   - 击打拍（LK 电机）由级联 PID 稳定在原位（编码器位置 `ready_position = 35000`）；
   - 大 Pitch 电机（DM 电机）抬起至上限 `PITCH_MAX = 0`；
   - 气动/舵机 PWM 处于抬起位（`pwm_up = 1100`）；
   - Stewart 基准推升高度就位（`z_min_push = 0.18m`）。
2. **`PUSHING`（推球上抛）**：
   - 联动启动 Push 状态机向上推球；
   - 检测到顶球机构动作完成并进入回程（`push_state_ == RESETTING`）后，推进至避让阶段。
3. **`LAYING_DOWN`（大 Pitch 机构倒伏避让）**：
   - 舵机/气动 PWM 切换为下压避让位（`pwm_down = 1900`）；
   - DM 电机驱动大 Pitch 向 `PITCH_MIN = -0.6 rad` 倒下；
   - **硬件位置安全互锁**：通过编码器检测到大 Pitch 实际位置确实达到 $\le -0.5\text{ rad}$（已移出挥拍扫掠扇区）后，记录时间戳并允许挥拍。
4. **`GOING_DOWN`（高速下挥拍击打）**：
   - 延迟计时满足 `strike_delay_time` 后，LK 击打电机直接注入全额开环大扭矩（`torque = -2000`），拍子极速下挥；
   - 编码器检测掠过最低击打点（`encoder > 50000`）后，转入后续顺势完成阶段。
5. **`GOING_UP`（向上顺势完成挥拍）**：
   - LK 击打电机持续保持扭矩（`torque = -2000`）；
   - 编码器检测到达挥拍结束阈值（`encoder < 55555`）后，宣布击打完成。
6. **`DONE`（复位与就绪）**：
   - LK 击打电机退出开环扭矩模式，切回串级 PID（外环角度环 + 内环速度环，内置最短弧度差算法与死区过滤），平滑制动并旋转回 `ready_position`；
   - 恢复左摇杆对 Stewart 并联平台的微调控制；
   - 当操作手将开关 **CH5** 拨回低电平（`< 1000`）后，状态机重置为 `WAITING`，等待下一次发球。

---

### 7. 电机与机构控制模式

- **LK 击打电机控制**：
  - **静态/复位**：外环角度 PID + 内环速度 PID 双闭环，带 `deadband = 300` 死区以及 65536 编码器单向最短旋转差值优化。
  - **击打瞬间**：直接切换为纯力矩输出（`torque = -2000`），实现最高角加速度。
- **DM 俯仰电机控制**：
  - 采用限速位置控制模式（`speed = 25 rad/s`），在 `[PITCH_MIN, PITCH_MAX]` 范围内平滑摆动。
