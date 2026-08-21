# 开机音量与偶发静音

## 当前设置

用户级 `default-volume.service` 调用 `/home/rc4/.local/bin/set-startup-volume`。
脚本等待指定的蓝牙音响 `MINISO-A127`（MAC `54:87:28:69:49:26`）创建 PulseAudio sink，解除该 sink 的静音，并把音量设置为 `30%`，对应桌面音量条截图中的大约三成位置。

该 service 是登录会话启动的常驻监听器。它启动时先检查一次目标 sink，随后通过 `pactl subscribe` 监听目标 sink 新建和默认输出切换事件。PulseAudio 重启导致订阅断开时，脚本会重新连接监听。

应用 unit 修改后执行：

```bash
systemctl --user daemon-reload
systemctl --user disable default-volume.service
systemctl --user enable default-volume.service
systemctl --user restart default-volume.service
```

## 已确认的静音原因

这条脚本本身不会把音量设为静音：它只对指定蓝牙 sink 执行 `pactl set-sink-mute ... 0`，再设置音量。它只响应目标 sink 新建和默认输出切换，不响应普通 sink 音量变化，因此不会持续覆盖手动调节。

本机一次启动的日志显示：

- 旧版 `default-volume.service` 在 `00:47:52` 已执行结束；
- 蓝牙设备 `MINISO-A127` 到 `00:52:11` 才完成连接并创建输出设备；
- PulseAudio 同时加载了 `module-switch-on-connect` 和 `module-device-restore`。

因此，蓝牙设备晚于旧版 service 出现时，PulseAudio 会把它切换为默认输出，并使用该设备保存的音量/静音状态。启动 service 之前解除静音的 HDMI 或其他设备，不会自动改变后来出现的蓝牙设备。新版监听器会在该蓝牙 sink 出现或成为默认输出时设置它；PulseAudio 重启、ALSA 管道断开、蓝牙断开重连后重新创建 sink 时也会重新监听。

## 排查命令

发现没有声音时，先在同一个桌面用户会话中执行：

```bash
pactl get-default-sink
pactl get-sink-volume @DEFAULT_SINK@
pactl get-sink-mute @DEFAULT_SINK@
pactl list short sinks
systemctl --user status default-volume.service --no-pager
journalctl --user -u default-volume.service -u pulseaudio.service -n 100 --no-pager
```

如果需要立即触发一次检测，可以重启监听器：

```bash
systemctl --user restart default-volume.service
```

若 `pactl get-sink-mute @DEFAULT_SINK@` 显示 `否` 但仍然没有声音，应继续检查输出设备是否选错、蓝牙设备是否处于连接状态，以及应用自己的音量；这时不是 PulseAudio 的 sink 静音状态。

目前在项目和用户启动配置中没有发现其他显式执行 `pactl set-sink-mute ... 1` 或 `amixer ... mute` 的启动命令。若后续仍出现“静音”，重点记录发生时间，并对照 PulseAudio/蓝牙日志；硬件静音键、桌面音量面板、应用自身静音和蓝牙设备端静音仍可能独立改变状态。监听器不会在普通静音事件后强制解除静音，这是为了保留主动静音的语义。
