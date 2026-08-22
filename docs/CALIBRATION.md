先启动相机，保持这个终端运行：
```bash
ros2 launch fruit_perception fruit_perception.launch.py
```

然后在另一个终端运行左相机标定：
```bash
ros2 run camera_calibration cameracalibrator \
  --size 11x8 \
  --square 0.2 \
  --camera_name left_camera \
  --ros-args \
  -r image:=/left_camera/image_raw \
  -r camera:=/left_camera
```
右相机标定命令：
``` bash
ros2 run camera_calibration cameracalibrator \
  --size 11x8 \
  --square 0.2 \
  --camera_name right_camera \
  --ros-args \
  -r image:=/right_camera/image_raw \
  -r camera:=/right_camera
```
