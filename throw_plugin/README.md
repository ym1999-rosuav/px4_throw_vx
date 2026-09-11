# Gazebo 抛飞仿真（PX4 Throw Launch）说明文档

本目录 `throw_plugin/` 是 PX4 固件仓库里为“抛飞”仿真做的 Gazebo 插件与相关改动。
目标场景（真机 / 仿真一致）：

> 飞机上电 → 手持 → 遥控器打内八解锁 → 地面站显示**定高(Altitude)模式**并等待
> → 把飞机抛出 → 无人机在任意姿态下迅速修正姿态 → 稳住定高。

真机没有 GPS，只有飞控、接收机、电机电调；仿真里先用默认 SITL（带模拟 GPS）把抛飞链路跑通。

---

## 1. 整体架构与数据流

```
┌──────────────────────────── Gazebo (gz sim, 进程名其实是 ruby) ───────────────────────────┐
│  world: Tools/simulation/gz/worlds/default.sdf                                            │
│  model: Tools/simulation/gz/models/x500_throw/model.sdf                                   │
│     └─ 每个 rotor 的 MulticopterMotorModel 系统                                          │
│     └─ ThrowImpulse 系统插件 (本目录编译出的 libThrowImpulse.so)                          │
│          • Subscribe("/throw")  ← 收到触发                                               │
│          • PreUpdate(): 给 base_link 的 ExternalWorldWrenchCmd 施加短时“力+力矩”冲量       │
│          • 仿真 IMU/气压/... 会被物理引擎算出                                             │
└───────────────▲───────────────────────────────────────────────┬──────────────────────────┘
                │ gz-transport topic (IMU/odom/baro/...)          │ /throw (gz.msgs.Empty)
                │                                                 │ gz topic -p ''  / 其它
┌───────────────┴───────────────────────────────────────────────┴──────────────────────────┐
│ PX4 SITL (bin/px4)                                                                        │
│  gz_bridge       : Gazebo ↔ uORB 传感器/电机 桥                                                 │
│  commander       : MulticopterThrowLaunch 状态机 + actuator_armed.lockdown               │
│  flight_mode_manager : 由姿态/高度 setpoint 生成 FlightTask（如 ManualAltitude）            │
│  mc_pos_control  : 位置/高度控制，输出姿态与推力                                           │
└──────────────────────────────────────────────────────────────────────────────────────────┘
```

触发抛飞的两种方式：

1. 本插件订阅 Gazebo 话题 `/throw`（推荐，脚本化方便）。
2. （旧方案，已废弃）`throw_plugin/throw_trigger.py` 直接给 gz 发速度命令。当前实现不使用它。

---

## 2. 我做的改动清单

### 2.1 固件（父仓库）

| 文件 | 改动 | 作用 |
|------|------|------|
| `src/modules/flight_mode_manager/FlightModeManager.hpp` | 增加 `#include <uORB/topics/actuator_armed.h>`、订阅 `_actuator_armed_sub`、成员 `_throw_lockdown_prev` | 监听“抛飞锁定”状态 |
| `src/modules/flight_mode_manager/FlightModeManager.cpp` | 在 `generateTrajectorySetpoint()` 里新增逻辑：当 `actuator_armed.lockdown` 由 true→false（电机启动）且处于解锁状态时，调用 `_current_task.task->reActivate()` | 抛出瞬间把高度/位置 setpoint **重置到当前状态**，否则飞机只会飞回解锁时的地面点 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_x500_throw` | 新增机型 | 设定 `COM_THROW_EN=1`、`COM_THROW_SPEED`、无 RC 时的参数、关闭不适用检查 |
| `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | 注册 `4023_gz_x500_throw` | 让 `make px4_sitl gz_x500_throw` 存在 |

`4023_gz_x500_throw` 关键参数：

```sh
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_throw}   # 用哪个 gz 模型
param set-default COM_THROW_EN 1             # 打开抛飞启动
param set-default COM_THROW_SPEED 4.0        # 判定“已抛出”的最小速度 m/s
param set-default COM_RC_IN_MODE 3           # 有 RC 用 RC，没有则接受 MAVLink 虚拟摇杆
param set-default FD_ESCS_EN 0               # 关掉 ESC 健康检查（SITL 会误报导致自动上锁）
param set-default FD_ACT_EN 0                # 关掉执行器故障检测（猛烈抛飞时避免误触发）
param set-default COM_DISARM_LAND -1         # 关掉落地自动上锁
```

### 2.2 Gazebo 模型（子模块 `Tools/simulation/gz`）

- 新增 `models/x500_throw/`：`model.sdf`（拷贝自 x500，末尾挂 `ThrowImpulse` 插件）、`model.config`、`LICENSE`、`thumbnails/`。
- `model.sdf` 中插件段落：

```xml
<plugin filename="libThrowImpulse.so" name="ThrowImpulse">
  <topic>/throw</topic>
  <velocity>7.0</velocity>       <!-- 抛出目标速度 m/s -->
  <acceleration>70.0</acceleration><!-- 冲量加速度 m/s^2，IMU 可见 -->
  <roll_rate>0.5</roll_rate>     <!-- 抛出时的初始角速度扰动 rad/s -->
  <pitch_rate>0.4</pitch_rate>
  <yaw_rate>0.3</yaw_rate>
</plugin>
```

### 2.3 插件（本目录）

- `throw_plugin/ThrowImpulse.cc`：Gazebo 系统插件，见第 4 节。
- `throw_plugin/CMakeLists.txt`：独立编译出 `libThrowImpulse.so`。
- `.bashrc` 里已有：`export GZ_SIM_SYSTEM_PLUGIN_PATH=~/PX4_ws/PX4-Autopilot/throw_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH`。

---

## 3. PX4 抛飞原理（读懂这段就懂一半）

### 3.1 MulticopterThrowLaunch 状态机

源码：`src/modules/commander/MulticopterThrowLaunch/MulticopterThrowLaunch.cpp`

```
DISABLED ──(COM_THROW_EN=1)──► IDLE
IDLE ──(armed)──► ARMED                                 打印 "Ready for throw launch"
ARMED ──(|v| > COM_THROW_SPEED)──► UNSAFE               打印 "Throw detected, motors will start once falling"
UNSAFE ──(vz > 0，即开始下落)──► FLYING                 打印 "Throw and fall detected, starting motors"
```

- 解锁后并不会立刻转桨，而是进入 **lockdown**（`actuator_armed.lockdown = isThrowLaunchInProgress()`）。
- 只有检测到“被抛出 + 开始下落”才让电机转。
- 这就是为什么仿真里触发抛飞后，`pxh` 里能看到上面两条 `MulticopterThrowLaunch` 日志。

### 3.2 为什么需要那几行固件改动

`COM_THROW_EN=1` 时，`TakeoffHandling::updateTakeoffState()` 会执行：

```cpp
if (armed && skip_takeoff) { _takeoff_state = TakeoffState::flight; }
```

也就是**一解锁就直接进入 flight**，跳过了正常起飞那段“持续重置 setpoint”的过程。

而 `FlightModeManager` 原本只在 `takeoff_state < RAMPUP` 时调用 `task->reActivate()`：

```cpp
if (_takeoff_state < takeoff_status_s::TAKEOFF_STATE_RAMPUP) {
    _current_task.task->reActivate();   // 会把高度 setpoint 重置到当前位置
}
```

于是问题来了：定高模式的高度 setpoint 在解锁那一刻被锁在**地面**，抛飞后飞机飞到 2~3 m，
控制器却认为“应该回到地面”，就会一路下沉。这表现为“抛飞能启动、姿态能修正，但高度稳不住”。

修复：监听 `actuator_armed.lockdown` 的 true→false 跳变（恰好是电机启动那一刻），
再 `reActivate()` 一次，把 setpoint 重置到当前高度：

```cpp
_actuator_armed_sub.update();
const actuator_armed_s &actuator_armed = _actuator_armed_sub.get();

if (_current_task.task && _throw_lockdown_prev && !actuator_armed.lockdown
    && (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
    _current_task.task->reActivate();
}
_throw_lockdown_prev = actuator_armed.lockdown;
```

> 注意：定高模式的任务（`FlightTaskManualAltitude`）只有在**遥控/虚拟摇杆有效**时才会正常运行
> （`_sticks.isAvailable()` 依赖 `manual_control_setpoint.valid`）。所以真机必须有遥控器，
> 仿真里请打开地面站虚拟摇杆，或让本插件之外的脚本发 MAVLink `MANUAL_CONTROL`。
> 这也是为什么“先切定高、再解锁、抛飞前后摇杆回中”能稳稳定高。

---

## 4. ThrowImpulse 插件实现解析

文件：`throw_plugin/ThrowImpulse.cc`

它实现两个 gz-sim 接口：

- `ISystemConfigure`：读 SDF 参数、用 `gz::transport::Node::Subscribe("/throw", ...)` 订阅话题。
- `ISystemPreUpdate`：每个仿真步被调用，检查是否有触发，施加冲量。

核心流程：

```cpp
void OnThrow(const gz::msgs::Empty &) {   // 收到 /throw 消息（可能来自别的线程）
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pending = true;
}

void PreUpdate(info, ecm) {
    if (info.paused || done) return;
    if (!launched) {
        // 取出 pending，解析模型/连杆，记录开始时间与质量/转动惯量
        link.SetAngularVelocity(ecm, {roll,pitch,yaw});   // 初始姿态扰动
    }
    gz::msgs::Wrench wrench;
    if (now - launchStart < impulseTime) {
        // 施加向上的力 + 力矩，每步“覆盖”而不是累加
        wrench.mutable_force()->set_z(mass * acceleration);
        wrench.mutable_torque()->set_x(ixx * rollRate  / impulseTime);
        ...
        applyWrench(ecm, wrench);
    } else {
        applyWrench(ecm, gz::msgs::Wrench());  // 清零，停止施力
        done = true;
    }
}
```

### 4.1 为什么用“力”而不是直接设速度

一开始我用 `link.SetLinearVelocity(0,0,7)` 直接给底盘一个速度。结果 **EKF 高度发散**：

- 物理引擎瞬间把速度改成 7 m/s，但 **IMU 没有测到对应的加速度**；
- PX4 的 EKF 融合 IMU / 气压 / GPS 时发现速度与加速度对不上，触发 z 轴重置，高度估计乱跳。

改成施加**短时力冲量**（约 0.12 s，加速度 70 m/s²）后，物理引擎积分出真实加速度，
IMU 测到的就是一致的加速度，EKF 不再发散。

### 4.2 为什么用 `ExternalWorldWrenchCmd` 且每步覆盖

`gz::sim::Link::AddWorldForce()` 是**累加**到 `ExternalWorldWrenchCmd` 组件上的。
如果每步都 `AddWorldForce`，力会越滚越大（实测直接把飞机射到 2 万米）。
所以这里直接对组件 `SetComponentData` **覆盖**成当前想要的力/力矩，冲量结束后再覆盖成 0。

---

## 5. 命令解析：`gz topic -t /throw -m gz.msgs.Empty -p ''`

这是 Gazebo Transport（gz-transport）自带的话题命令行工具，和 ROS 2 无关，是 Gazebo 自己的发布/订阅总线
（基于 ZeroMQ + UDP，默认用 `GZ_PARTITION`/`GZ_IP` 划分网络）。

逐段拆开：

| 片段 | 含义 |
|------|------|
| `gz` | Gazebo CLI 主命令 |
| `topic` | 子命令：操作 Gazebo 话题 |
| `-t /throw` | `--topic`，话题名 `/throw`，必须与插件里 `Subscribe("/throw", ...)` 一致 |
| `-m gz.msgs.Empty` | `--msgtype`，消息类型。`gz.msgs.Empty` 是 protobuf 定义的空消息（**没有任何字段**） |
| `-p ''` | `--pub`，发布消息；后面跟的是 protobuf 的 TextFormat 文本。因为 `Empty` 没有字段，所以给空字符串 `''` |

合起来：**往 Gazebo 的 `/throw` 话题发布一条“空消息”**。
插件在 `OnThrow()` 里收到后把 `pending=true`，下一个仿真步就施加抛飞冲量。

关键点：

- **必须带 `-p`**。你之前笔记里的 `gz topic -t /throw -m gz.msgs.Empty` 只指定了话题和类型却没给“命令”，
  工具不知道你要干嘛，于是打印 usage。`-p ''` 里的 `''` 是一个**空字符串参数**，作用是选中“发布”这个动作。
- 为什么用 `Empty`：这里只需要一个“事件/信号”，不需要携带数据，省事又明确。
- 等价写法：`gz topic -t /throw -m gz.msgs.Empty -p ' '`（不同版本对空 TextFormat 的容忍度略有差异），
  用 `-p ''` 最稳。
- 分区/网络：如果启动仿真时设了 `GZ_PARTITION` 或 `GZ_IP`，发命令的终端也要有相同环境变量，
  否则两个进程“不在同一个总线里”，消息收不到。本项目的启动设了 `GZ_IP=127.0.0.1`。

常用配套命令：

```bash
gz topic -l                       # 列出所有话题
gz topic -i -t /throw             # 查看某话题的类型/发布者/订阅者
gz topic -e -t /throw             # 回显话题内容
gz topic -f -t /world/default/clock   # 查看某话题频率
```

---

## 6. 编译与运行

### 6.1 编译

```bash
# 1) 编译 Gazebo 插件（独立 cmake）
cd ~/PX4_ws/PX4-Autopilot/throw_plugin
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4            # 产出 build/libThrowImpulse.so

# 2) 编译 PX4 SITL（含机型与固件改动）
cd ~/PX4_ws/PX4-Autopilot
make px4_sitl_default -j4
```

确保当前终端已 `source ~/.bashrc`（里面有 `GZ_SIM_SYSTEM_PLUGIN_PATH` 指向插件 build 目录）。

### 6.2 运行（带 GUI）

```bash
cd ~/PX4_ws/PX4-Autopilot
make px4_sitl gz_x500_throw
```

Gazebo 会启动并生成 `x500_throw_0` 模型，同时 PX4 启动。

在 PX4 的 `pxh>` 终端里（真机对应遥控器/地面站）：

```
commander mode altctl       # 先切定高（务必在解锁前）
commander arm               # 健康检查挡的话用 commander arm -f
```

在另一个终端触发抛飞：

```bash
gz topic -t /throw -m gz.msgs.Empty -p ''
```

用地面站虚拟摇杆时：打开 QGC 虚拟摇杆即可（`COM_RC_IN_MODE=3` 会锁到 MAVLink），
抛飞前后摇杆回中，飞机会在抛飞恢复的高度上定高。

### 6.3 headless 快速验证

```bash
cd ~/PX4_ws/PX4-Autopilot/build/px4_sitl_default/rootfs
HEADLESS=1 PX4_SIM_MODEL=gz_x500_throw GZ_IP=127.0.0.1 \
  GZ_SIM_SYSTEM_PLUGIN_PATH=~/PX4_ws/PX4-Autopilot/throw_plugin/build: \
  ../bin/px4
```

---

## 7. 期望现象与实测

抛出后（带 17°/13° 初始横滚/俯仰扰动）约 1 秒内姿态回正，高度锁定：

```
t=10.7s  抛出     rel_alt=2.74m  Roll=17.3°  Pitch=-13.2°
t=11.8s  姿态回正  Roll=-0.2°   Pitch=1.9°
t=12.9~35s 稳定   rel_alt≈2.74m(±0.02)  Roll/Pitch≈0
pxh> [MulticopterThrowLaunch] Throw detected, motors will start once falling
pxh> [MulticopterThrowLaunch] Throw and fall detected, starting motors
pxh> [flight_mode_manager] Throw launch released: re-initialized flight task setpoint
```

---

## 8. 参数调优

| 位置 | 参数 | 说明 |
|------|------|------|
| `model.sdf` | `velocity` | 抛出目标速度，要 > `COM_THROW_SPEED` |
| `model.sdf` | `acceleration` | 冲量加速度，越大越快到位、IMU 冲击越大 |
| `model.sdf` | `roll_rate/pitch_rate/yaw_rate` | 初始角速度扰动，用来演示“任意姿态修正” |
| 机型文件 | `COM_THROW_SPEED` | 抛飞判定阈值，默认 5，本项目设 4 |
| 机型文件 | `MPC_THR_HOVER` | 悬停油门，x500 常用 0.60 |

---

## 9. 故障排查

1. **改了模型/插件没生效**：很可能是残留的 Gazebo 在跑，PX4 复用了旧世界。
   `gz sim` 的进程名其实是 **`ruby`**，所以 `killall gz` 杀不掉！请用：
   ```bash
   pkill -9 -f '[g]z sim'
   pkill -9 -x px4
   ```
2. **`gz topic ... -p ''` 报 usage**：漏了 `-p`，见第 5 节。
3. **高度乱飞/发散**：不要用“直接设速度”，用力冲量（本插件已实现）。
4. **解锁被拒 `Arming denied`**：EKF 未收敛或健康检查；等几秒，或用 `commander arm -f`；
   SITL 里可关 `FD_ESCS_EN`（本项目机型已关）。
5. **收不到 `/throw`**：检查 `GZ_PARTITION`/`GZ_IP` 是否与启动仿真时一致。

---

## 10. 后续可做

- 无 GPS 版本：把 `EKF2_GPS_CTRL=0`、`COM_ARM_WO_GPS=1`，只用气压定高、姿态控制，
  更贴近真机（真机没有 GPS）。
- 机臂弹开：在抛出后给折叠关节加旋转（纯视觉/动力学近似），本项目暂未做。
- 把模型做成 `default.sdf` 内置，方便纯 `gz sim` 手动观察。
