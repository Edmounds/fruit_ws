# Q1-Q9 修复执行

用户已授权直接修复。范围限于 review.md 的 Q1-Q9。
保留旧修复和用户工作，不重启服务，不发送实车运动命令，不推送。

## 阶段

- [x] Q1/Q2：已实现，等待公开端点检查。
- [x] Q4/Q8：已实现，等待副本文档检查。
- [x] Q3/Q9：已实现，删除 C++ 标定/姿态变换副本。
- [x] Q5：已实现，HTTP 启动会话和精确 JSON 确认，等待并发检查。
- [x] Q6/Q7：已实现锁存 marker、按需订阅和前置准入，等待真实组件检查。
- [x] 定向构建、已有测试、公开端点无执行器检查和 ABI。
- [x] 文档、验证提交、最新 main 集成及交接。

## 当前基线

fruit_ws/main：8c04ad7bffa623fd1088d6d3700c1411bf8d8de5。
任务 worktree：.worktrees/robviz-web，task/robviz-web，4b18861dfaefe062b7f46392ac75d228a79d78b6。
RobViz：/home/rc4/robviz，task/waypoints-web，6ce930234e4e40ff61a2377283165e11b800a72a。
任务现有六个机器人文件和三个前端文件的 dirty 修改与上轮记录一致。
未发现新的外部提交。主目录 Realsense 脏状态不属于本任务。

## 验证边界

检查使用 tmp 文档和 ROS_DOMAIN_ID 隔离域，不连接真实执行器。
禁止私有回调/mock 检查。硬件、生产页面和真实吞吐仍需单独监督验收。
构建使用独立 tmp build/install/log，避免主 CMake cache 冲突。

## Next Step

软件修复已完成。生产构建、服务重启和实车验收需要另行授权。

## 验证记录

- compileall、定向 flake8、C++ uncrustify 通过。
- 四包隔离构建通过，修改后的 bridge/recorder 已再次构建。
- 前端 lint/build 通过，31 文件、161 测试通过。
- Vitest 有既有大 chunk 和退出等待告警，未将其称为浏览器验收。
- 现有 recorder 测试 6 项通过。
- ABI 第一次使用了错误库名，后续改用 CMake 中的 `libfruit_robviz_bridge_component.so`。
- overlay source 在 `set -u` 下失败，已改为 source 完毕再启用 `set -u`。
- 公开端点脚本位于 ignored `tmp/robviz_repair_check.py`，使用副本文档和 ROS_DOMAIN_ID=197。
- 纯算法检查 32 项通过，当前隔离 perception 结果为 24 tests、0 failures。
- 两个组件 ABI 均为系统 OpenCV 4.5.4，无缺失库或 `/usr/local/lib/libopencv*`。
- 公开端点检查通过：采样 1 秒、严格丢弃确认、加载失败保留 Undo、晚订阅 Marker。
- 取消后的重复旧时间戳不能补足窗口，未知 Action 结果保持 active/unknown 且游标不变。
- Python 节点成功加载实际普通 YAML，Base 与共享变换一致，运动立即无效，旧图像不套用新位姿。
- C++ 实际组件成功加载到 intra-process 容器，两路 JPEG 尺寸/颜色、全零/常量深度、预算/FPS 和关闭撤订阅检查通过。
- 本轮离线输入由独立域中的合成发布者提供，不是生产相机性能验收。
- 初次未知 Action 夹具缺少 status/feedback 发布端，补齐公开协议端点后通过。
- 初次 Python 配置读取落到未集成 main 的旧 recorder YAML，已通过公开参数指定 worktree YAML。
- RobViz 已提交 `8a608f0`，6 文件，未合并或推送。
- 机器人修复已提交 `47afca1`，15 文件。
- 私有任务分支已适配 main `8c04ad7`，新 tip 为 `fa9ad73`。
- 变基前后业务内容完全相同，差异仅为 main 的 review.md。
- 已 fast-forward 集成到 `fruit_ws/main`，修复提交为 `fa9ad73`。
- 主目录追加验证提交 `b195fc8`，对齐硬件停稳阈值和 `check_preview` 安装名称。
- 最新 main 使用独立 `tmp/robviz-main-{build,install,log}` 重新构建四包并通过公开端点检查。
- 最终检查包含两个并发重载、精确版本/会话确认、陈旧参数拒绝和实际组件加载。
- 主目录现有算法检查 32 项通过，隔离 perception 检查 24 项通过。
- 主目录 compileall、flake8、uncrustify、shell syntax、diff check 和两组件 ABI 均通过。
- 日志：`/tmp/pi-unified-exec-30-3bcfef4e.log`、`/tmp/pi-unified-exec-31-8f3072b8.log`。
- `git worktree remove`、`git branch -d` 和 `git worktree prune` 已成功清理机器人任务 worktree。
- 已保留 Realsense 外部脏状态及历史未跟踪计划、基线和 review 备份。
- 未推送、部署、更新 `/etc`、修改主 `install/` 或重启生产服务。

## 结果与范围

- Q1：未知接受/派发/结果保留导航所有权，取消终态后重新确认真实样本窗口。
- Q2：采样期间不持有长锁，1 秒采样同时处理里程计更新。
- Q3：Python 复用共享变换，按采集时间选择连续停稳历史，不使用最新位姿替代旧位姿。
- Q4：HTTP、recorder 加载和保存校验均绑定同一字节快照及 hash。
- Q5：单次重载在途，JSON 精确确认版本/会话，旧参数不能回写。
- Q6：MarkerArray 锁存当前完整快照。
- Q7：首次配置前和无订阅时不接收 raw，FPS 准入先于昂贵处理。
- Q8：丢弃确认严格校验布尔值，候选文件校验成功后才替换内存。
- Q9：标准 YAML 读取配置，C++ 不再使用 OpenCV FileStorage 解析生产标定。

机器人修复范围为 15 个已有任务文件。
前端修复范围为 6 个已有任务文件，提交 `8a608f0` 留在 `task/waypoints-web`。
机器人原功能提交也已集成到 main，入口变化尚未部署到运行服务。
本文件是本轮修复交接，原 plan/progress/review 保留历史状态。

## 重跑无执行器检查

临时脚本和副本数据保留在 ignored `tmp/`，未提交。
检查只启动合成发布者、公开协议端点和真实预览组件，不启动驱动或执行器。

```bash
cd /home/rc4/fruit_ws
source scripts/setup_fruit_runtime.bash
source tmp/robviz-main-install/local_setup.bash
export ROS_DOMAIN_ID=197
export ROS_LOG_DIR=/home/rc4/fruit_ws/tmp/robviz-repair-check/ros
/usr/bin/python3 -u tmp/robviz_repair_check.py
```

独立构建命令：

```bash
colcon --log-base tmp/robviz-main-log build --base-paths src \
  --build-base tmp/robviz-main-build --install-base tmp/robviz-main-install \
  --packages-select fruit_perception fruit_waypoint_recorder fruit_robviz_bridge fruit_nav_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

## 尚未完成的运行验收

离线检查不证明真实图像、生产浏览器、实际运动取消或比赛推理性能。
原计划的 60 秒带宽/吞吐对比和 10 分钟稳定性检查仍未执行。
Firefox headless 无 WebGL 的历史限制仍保留。
实际录点、单段导航和取消必须由现场操作者监督。
生产运行前必须重新构建主 `install/` 并在安全状态下重启受影响节点。
不得把源码集成或临时 overlay 构建视为部署完成。
