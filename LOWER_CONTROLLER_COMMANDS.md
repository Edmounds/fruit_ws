# 下位机控制台常用命令

先启动控制台：

```bash
source /home/rc4/fruit_ws/setup_fruit_runtime.bash
/usr/bin/python3 /home/rc4/fruit_ws/lower_controller_console.py
```

进入 `fruit> ` 后输入命令。

## 状态命令

通用格式：

```text
task <task_id> <task_status>
```

例如发送 `task_id=4, task_status=0`：

```text
task 4 0
```

它也可以写成快捷命令：

```text
qr-pose
```

常用状态命令：

```text
grip              # task 0 0，夹爪抓取
open              # task 0 1，夹爪打开
gimbal-down       # task 1 0，云台向下
gimbal-up         # task 1 1，云台向上
left              # task 2 0，左侧观察位置
right             # task 2 1，右侧观察位置
qr-pose           # task 4 0，二维码识别姿态
task 6 0          # 返回初始姿态
task 7 0          # 机械臂收回姿态
```

## 区域命令

```text
area A            # 等价于 task 5 0
area B            # 等价于 task 5 1
area C            # 等价于 task 5 2
area D            # 等价于 task 5 3
```

## 机械臂目标

坐标单位是米：

```text
arm <x> <y> <z> <z_type>
```

示例：

```text
arm 0.30 0.10 0.20 1
```

## 速度命令

发送一帧速度：

```text
vel <linear_x> <angular_z>
```

示例：

```text
vel 0.20 0.00
vel 0.00 0.50
```

发送停止命令：

```text
stop
```

## 其他

```text
help              # 查看命令帮助
quit              # 退出控制台
```

控制台会自动发送心跳，但默认隐藏心跳日志。需要查看心跳时，启动时加：

```bash
/usr/bin/python3 /home/rc4/fruit_ws/lower_controller_console.py --show-heartbeat
```

区域命令也可以一次性发送，例如：

```bash
/home/rc4/fruit_ws/zone_witch.sh --A
```
