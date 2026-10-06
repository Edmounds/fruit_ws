# 下位机 AI 修改指南：TrajectoryPoint 途经点标记

本文档提供给负责下位机机械臂控制代码的开发者和 AI 编程代理。
本文档描述一次协议扩展的实施要求。上位机和下位机必须按同一版协议构建和部署。

**当前状态：上位机源码和协议配置已经加入 `is_via_point` 字段。
下位机仍需同步生成协议代码并更新业务固件后，双方才能启用该功能。**

本文档只增加“该轨迹点是否为途经点”的标记。
本文档不改变 IK、关节映射、轨迹时间轴、夹爪协议、急停逻辑或硬件限位逻辑。

## 1. 要实现的行为

上位机发送的每个 `TrajectoryPoint` 增加一个 `is_via_point` 字段。

| 值 | 名称 | 下位机处理 |
| ---: | --- | --- |
| `0` | 精确到达点 | 使用正常的严格到位容差 |
| `1` | 途经点 | 使用较大的途经点容差，允许轨迹继续通过 |

上位机只发送模式标记，不发送具体的毫米容差值。
下位机为精确点和途经点分别使用已有或新增的本地容差配置。

途经点容差只影响轨迹点的到位判定。
它不能放宽关节软限位、硬件保护、碰撞保护、急停或通信失联保护。

## 2. 权威修改顺序

修改前先确认当前分支和协议版本。不要直接修改生成文件。

上位机侧的权威源文件是：

```text
src/fruit_hardware_interfaces/msg/TrajectoryPoint.msg
src/auto_serial_bridge/config/protocol.yaml
src/fruit_arm_planner/fruit_arm_planner/trajectory_planner.py
src/fruit_arm_planner/fruit_arm_planner/arm_planner_node.py
src/fruit_arm_planner/config/arm_presets.yaml
```

下位机侧的权威输入是上位机 `auto_serial_bridge` 根据协议 YAML 生成的：

```text
protocol.h
protocol.c
PROTOCOL_DOC.md
```

这些文件属于生成产物。不要在 `build/` 或 `install/` 中直接编辑它们。

正确流程是：

1. 上位机接口和 `protocol.yaml` 先完成修改。
2. 重新生成上位机 ROS 绑定和 MCU 协议文件。
3. 将新的 `protocol.h`、`protocol.c` 同步到下位机工程。
4. 修改下位机业务接收逻辑。
5. 编译下位机并完成离线协议测试。
6. 再构建和部署上位机桥接、接口和 planner。

## 3. 线协议字段

消息名称和消息 ID 不变：

```text
消息：TrajectoryPoint
ID：  0x11
方向：ROS 发送到 MCU
可靠传输：是
```

新的业务字段顺序如下：

| 顺序 | 字段 | 类型 | 单位或取值 |
| ---: | --- | --- | --- |
| 1 | `trajectory_id` | `uint32_t` | 轨迹 ID |
| 2 | `point_id` | `uint16_t` | 点 ID |
| 3 | `time_from_start` | `uint32_t` | 毫秒 |
| 4 | `joint_1` | `float` | 硬件角，rad |
| 5 | `joint_2` | `float` | 硬件角，rad |
| 6 | `joint_3` | `float` | 硬件角，rad |
| 7 | `joint_4` | `float` | 硬件角，rad |
| 8 | `gripper_action` | `uint8_t` | `0=HOLD, 1=OPEN, 2=CLOSE` |
| 9 | `is_via_point` | `uint8_t` | `0=精确，1=途经` |

`is_via_point` 放在现有 `gripper_action` 后面。
不要调整现有字段顺序，也不要把它插入四个关节角之间。

当前业务 payload 为 27 字节。
增加字段后，业务 payload 为 28 字节。

`TrajectoryPoint` 启用可靠传输，因此线协议还会自动追加 1 字节可靠序号。
可靠 payload 将变为 29 字节。

可靠序号不是 `Packet_TrajectoryPoint` 业务结构体字段。
下位机业务代码不要读取或修改该序号。

## 4. 下位机结构体和常量

重新生成协议后，生成的 `Packet_TrajectoryPoint` 应包含：

```c
uint32_t trajectory_id;
uint16_t point_id;
uint32_t time_from_start;
float joint_1;
float joint_2;
float joint_3;
float joint_4;
uint8_t gripper_action;
uint8_t is_via_point;
```

下位机业务代码应使用明确的常量：

```c
#define POINT_EXACT 0u
#define POINT_VIA   1u
```

如果下位机工程已经有统一的协议常量命名规则，应使用现有规则，但必须保持数值一致。

不要使用“非零就是途经点”的宽松处理。
建议只接受 `0` 和 `1`，收到其他值时拒绝当前轨迹或进入现有的轨迹失败流程。

## 5. 上位机发送规则

### 5.1 预设 waypoint

预设 waypoint 支持可选字段：

```yaml
is_via_point: true
```

规则如下：

| YAML 配置 | 发送值 |
| --- | ---: |
| 没有 `is_via_point` | `0` |
| `is_via_point: false` | `0` |
| `is_via_point: true` | `1` |

该字段只作用于对应 waypoint 的目标点。
它不是整个 preset 的全局设置。

例如：

```yaml
waypoints:
  - target: [0.20, 0.0, 0.25, 0.0]
    duration_ms: 1000
    is_via_point: true

  - target: [0.40, 0.0, 0.25, 0.0]
    duration_ms: 1000
    is_via_point: false
```

第一个目标是途经点。
第二个目标是精确到达点。

### 5.2 自动生成的插值点

上位机在两个目标之间生成的 PTP 或 LIN 插值点，默认发送：

```text
is_via_point = 1
```

这包括：

- PTP 的关节插值点。
- LIN 的笛卡尔插值点。
- 多 waypoint 预设中自动生成的中间点。

插值点不因为它恰好具有一个合法的关节角，就变成精确到达点。
它们只用于连续执行轨迹。

### 5.3 明确目标点

没有开放 `is_via_point` service 字段。
因此 `move_cartesian` 和 `move_joints` 的最终目标点默认发送：

```text
is_via_point = 0
```

这表示 mission 或控制台通过 service 请求的最终目标必须精确到达。

### 5.4 第 0 个起始点

轨迹第 0 个点是上位机冻结的新鲜实测硬件角：

```text
point_id = 0
time_from_start = 0
gripper_action = HOLD
```

该点建议发送：

```text
is_via_point = 0
```

它是轨迹的实测起始锚点，不是由目标之间插值得到的运动点。

### 5.5 驻留点

预设 waypoint 可能包含夹爪动作和 `dwell_ms`。
驻留点应继承对应 waypoint 的 `is_via_point` 值。
途经点可以同时携带 OPEN、CLOSE 或驻留动作。
下位机应保持夹爪动作语义，并仅根据 `is_via_point` 选择当前点的到位容差。

## 6. 下位机到位判定

下位机只修改当前轨迹点的到位容差选择。
不要修改当前的时间插值、点序、轨迹 ID 或关节控制算法。

推荐逻辑：

```c
switch (point.is_via_point) {
  case POINT_EXACT:
    tolerance = exact_point_tolerance;
    break;

  case POINT_VIA:
    tolerance = via_point_tolerance;
    break;

  default:
    fail_current_trajectory(INVALID_TRAJECTORY_POINT);
    break;
}
```

其中：

```text
exact_point_tolerance：现有精确点容差
via_point_tolerance：新增或已有的较大途经点容差
```

途经点容差必须大于或等于精确点容差。
具体数值必须根据机械臂实测误差、控制周期和比赛安全要求确定。
本文档不规定具体毫米数。

途经点模式不得执行以下行为：

- 放宽关节软限位。
- 放宽电机电流或温度保护。
- 关闭通信超时保护。
- 忽略急停或 STOP。
- 修改轨迹时间戳。
- 跳过 `point_id` 检查。
- 把下一个点提前当作当前点执行。
- 改变夹爪动作语义。

途经点只改变“当前点是否必须严格到位”的判定阈值。

## 7. 轨迹缓存要求

轨迹缓存中的每个点都必须保存 `is_via_point`。
不要只在轨迹级别保存一个全局标志。

错误做法：

```c
trajectory.is_via = true;
```

这种做法无法表达同一条轨迹中同时存在精确点和途经点。

正确做法是让缓存中的每个点保留自己的标记：

```c
trajectory.points[i].is_via_point
```

轨迹执行到 `point_id=i` 时，使用该点自己的标记选择容差。

保留现有校验：

- `trajectory_id` 必须匹配当前轨迹。
- `point_id` 必须连续。
- 第 0 点必须是 `time_from_start=0`。
- 时间戳必须严格递增。
- 点数不能超过 `point_num` 和缓存容量。
- 四个关节角必须是有限值。
- 关节角必须通过现有硬件限位检查。

## 8. 与可靠传输的关系

`TrajectoryPoint` 仍然使用可靠发送和 ACK/重传。
新增字段不改变 ACK、重传、帧头、CRC 或可靠序号规则。

收到重复的 `TrajectoryPoint` 时，仍按现有 `trajectory_id`、`point_id` 和可靠序号规则处理。
不要因为 `is_via_point` 不同而重复追加轨迹点。

如果同一个 `trajectory_id` 和 `point_id` 收到两份内容不同的点，必须沿用现有冲突处理规则。
如果现有代码没有冲突处理规则，应拒绝当前轨迹并记录完整字段。

## 9. 协议版本和旧固件

新增字段会改变 `TrajectoryPoint` 的 payload 长度和协议哈希。
旧版下位机不能直接当作新版下位机使用。

当前上位机协议配置为：

```yaml
require_handshake: false
```

因此不能依赖握手自动拦截新旧版本混用。
部署时必须同时更新：

```text
上位机 auto_serial_bridge
上位机 fruit_hardware_interfaces
上位机 fruit_arm_planner
下位机 protocol.h / protocol.c
下位机 TrajectoryPoint 缓存结构
下位机到位判定代码
```

不要在下位机业务回调中通过 payload 长度猜测“旧版就是精确点”。
如果确实需要兼容旧协议，必须单独设计协议版本或长度兼容方案，并完成双方测试。
不能把兼容逻辑隐藏在普通轨迹处理代码中。

## 10. AI 修改工作流

给下位机 AI 的执行指令应按以下顺序进行。

### 第一步：定位真实源文件

先确认下位机工程中：

1. `TrajectoryPoint` 的生成结构体来自哪里。
2. `on_receive_TrajectoryPoint()` 或同等接收回调在哪里。
3. 轨迹点缓存结构在哪里。
4. 轨迹点到位判定在哪里。
5. 精确点容差当前配置在哪里。
6. STOP、ESTOP、硬件限位和通信超时保护在哪里。

不要根据函数名猜测文件。
先搜索 `TrajectoryPoint`、`trajectory_id`、`point_id`、`time_from_start`、`gripper_action`。

### 第二步：更新协议生成产物

使用上位机提供的新版本 `protocol.h` 和 `protocol.c`。
不要手动在生成文件中插入字段。

确认生成结果包含：

```c
uint8_t is_via_point;
```

确认 `TrajectoryPoint` 的 payload 长度增加 1 字节。

### 第三步：更新下位机缓存

将 `is_via_point` 从接收结构体复制到每一个缓存点。
确认复制过程不会丢失该字段。

重点检查：

- Header 接收后的缓存清空逻辑。
- Point 接收和复制逻辑。
- 轨迹点数组或环形缓存。
- 轨迹启动时的点读取逻辑。
- 重传或重复包处理逻辑。

### 第四步：更新到位判定

在读取当前轨迹点时，根据该点的 `is_via_point` 选择容差。
只改变当前点的到位判定阈值。

不要复制一套新的轨迹控制器。
不要把容差判断移到串口接收回调中。
串口接收回调只负责接收、校验和入队。

### 第五步：加入配置和日志

建议增加两个明确的本地配置：

```text
exact_point_tolerance
via_point_tolerance
```

启动时打印两者的单位和数值。
执行轨迹点时至少在调试日志记录：

```text
trajectory_id
point_id
is_via_point
selected_tolerance
```

不要在默认信息日志中输出每个 20 Hz 控制周期的高频数据。

### 可直接交给下位机 AI 的任务提示

可以将以下内容作为下位机 AI 的任务说明：

```text
请按 docs/LOWER_CONTROLLER_VIA_POINT_UPDATE_GUIDE.md 修改下位机机械臂轨迹处理。

先搜索并阅读 TrajectoryPoint、trajectory_id、point_id、time_from_start、
gripper_action、轨迹缓存、点到位判定、STOP、ESTOP、硬件限位和通信超时相关代码。
不要猜测文件名，不要直接编辑自动生成文件。

使用上位机提供的新 protocol.h 和 protocol.c。
确认 Packet_TrajectoryPoint 新增 uint8_t is_via_point，且业务 payload 增加 1 字节。
将该字段保存到每一个轨迹缓存点。

is_via_point=0 使用现有精确点容差。
is_via_point=1 使用途经点容差。
只修改点到位判定阈值，不修改 IK、关节映射、时间插值、点序、限位、STOP、ESTOP、
ACK、重传和通信超时逻辑。

只接受 is_via_point=0 或 1。
收到其他值时，按现有轨迹失败流程处理。
不要使用非零即途经点的宽松判断。
不要把该字段保存为轨迹级全局标志。

保留所有既有安全保护。

完成后报告：修改文件、协议 payload 长度、精确点容差、途经点容差、非法值处理、
夹爪动作和驻留语义、离线测试结果和实机测试限制。
```

## 11. 离线验收标准

在连接真实机械臂前，至少完成以下测试。

### 协议解析

- 新版 `TrajectoryPoint` 能正确解析 28 字节业务 payload。
- 可靠序号仍能正确剥离。
- `is_via_point=0` 能正确解析。
- `is_via_point=1` 能正确解析。
- `is_via_point=2` 被拒绝。
- 旧版 27 字节 payload 的处理符合双方明确的版本策略。

### 轨迹缓存

- 一条轨迹可以同时包含精确点和途经点。
- 每个缓存点保留自己的标记。
- 重复可靠包不会重复追加点。
- 点 ID 和时间戳规则保持不变。

### 容差选择

给定相同目标误差时：

- 精确点使用精确容差。
- 途经点使用途经容差。
- 途经点不会修改安全限位。
- 途经点不会修改 STOP/ESTOP 行为。

### 夹爪动作和驻留

- `is_via_point=1` 和 OPEN 保留夹爪动作，并使用途经点容差。
- `is_via_point=1` 和 CLOSE 保留夹爪动作，并使用途经点容差。
- `is_via_point=1` 和驻留动作保留驻留语义，并使用途经点容差。
- 精确点的 HOLD、OPEN、CLOSE 行为保持原样。

## 12. 实机验收顺序

实机验证必须从低速、小范围、无夹爪动作开始。

1. 发送一条只包含精确点的短轨迹。
2. 确认下位机仍能正常进入 RUNNING 和 COMPLETED。
3. 发送包含一个途经点和一个精确终点的轨迹。
4. 查看日志确认两个点选择了不同容差。
5. 确认最终精确点仍然严格到位。
6. 确认轨迹 ID、点 ID、时间轴和状态上报没有变化。
7. 再验证夹爪动作和驻留点。
8. 最后验证 STOP 和 ESTOP。

出现以下任一情况时，停止实机测试：

- 新旧协议 payload 长度不一致。
- 途经点标记在缓存后丢失。
- 精确点使用了途经点容差。
- 途经点放宽了硬件限位。
- 点 ID 或时间戳发生变化。
- STOP 或 ESTOP 不能立即按原规则处理。
- 下位机没有上报明确的 FAILED 或 EMERGENCY 状态。

## 13. 禁止事项

下位机 AI 不得执行以下修改：

- 不得修改 `TrajectoryHeader` 的字段含义。
- 不得修改 `trajectory_id`、`point_id` 或时间单位。
- 不得把四个关节角从 rad 改为 degree。
- 不得在下位机重新实现 IK。
- 不得根据 `is_via_point` 修改关节映射或零位偏置。
- 不得删除原有硬件限位。
- 不得用途经点标记屏蔽急停、STOP 或通信保护。
- 不得修改可靠 ACK 和重传规则。
- 不得把 `is_via_point` 改成轨迹级全局配置。
- 不得手动编辑自动生成的协议文件作为长期修复。
- 不得宣称 ACK 等于机械臂到位。

完成修改后，必须把以下信息交给上位机负责人：

```text
下位机协议版本或 PROTOCOL_HASH
TrajectoryPoint 业务 payload 长度
精确点容差及单位
途经点容差及单位
非法 is_via_point 值的处理方式
途经点与夹爪动作、驻留的处理方式
离线测试结果
实机测试结果和限制
```
