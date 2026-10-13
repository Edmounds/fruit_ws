#!/usr/bin/env python3
"""Interactive terminal checker for B/C QR codes directly from camera RGB stream."""

import sys
import time
from typing import Any, Optional, Tuple

import cv2
import numpy as np
from pyzbar.pyzbar import decode as decode_barcodes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from fruit_qr_planner.planner import (
    build_b_plan,
    build_c_plan,
    normalize_payload,
    ParseError,
)
from fruit_qr_planner.qr_node import image_to_bgr, select_unique_payload


GREEN = "\033[1;32m"
RED = "\033[1;31m"
YELLOW = "\033[1;33m"
CYAN = "\033[1;36m"
BOLD = "\033[1m"
RESET = "\033[0m"


class QrTerminalMonitor(Node):
    def __init__(self) -> None:
        super().__init__("qr_terminal_monitor")
        self._image_topic = "/camera/color/image_raw"
        self._subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._fallback_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )
        self._compressed_pub = self.create_publisher(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            qos_profile_sensor_data,
        )
        self._annotated_compressed_pub = self.create_publisher(
            CompressedImage,
            "/fruit_perception/camera/annotated_image/compressed",
            qos_profile_sensor_data,
        )
        self._last_report_payload: Optional[str] = None
        self._last_report_time: float = 0.0
        self._last_compress_pub_time: float = 0.0
        self._frame_count = 0
        self._total_frames = 0
        self._last_fps_time = time.time()
        self._fps = 0.0
        self._last_heartbeat_time = 0.0
        self._start_time = time.time()
        self._status_timer = self.create_timer(1.0, self._on_status_timer)

        sep = "=" * 70
        print(f"\n{BOLD}{CYAN}{sep}{RESET}")
        print(f"{BOLD}{CYAN} [Fruit QR Test] 比赛流程同款二维码检验终端{RESET}")
        print(f"{CYAN} 监听话题: {self._image_topic}{RESET}")
        print(f"{CYAN} 检验规则: B区严格8棵树(苹果/梨子) | C区严格8点位(1~12不重复){RESET}")
        print(f"{CYAN} 提示: 请将二维码置于摄像头正前方 (建议距离 30~50cm, 避免反光/虚焦){RESET}")
        print(f"{BOLD}{CYAN}{sep}{RESET}\n")

    def _on_status_timer(self) -> None:
        now = time.time()
        if self._total_frames == 0 and now - self._start_time >= 1.5:
            sys.stdout.write(
                f"\r\033[K{YELLOW}[等待中]{RESET} 正在等待相机图像流 ({self._image_topic})... 请确认摄像头正常连接"
            )
            sys.stdout.flush()

    def _decode_and_annotate(self, image: Any) -> Tuple[Optional[str], Any]:
        payloads = []
        annotated = image.copy()
        for barcode in decode_barcodes(image):
            if barcode.type != "QRCODE":
                continue
            text = None
            try:
                text = barcode.data.decode("utf-8")
            except UnicodeDecodeError:
                try:
                    text = barcode.data.decode("gbk")
                except UnicodeDecodeError:
                    pass
            if text is not None and text.strip():
                payloads.append(text)
            if barcode.polygon:
                pts = np.array([[p.x, p.y] for p in barcode.polygon], dtype=np.int32)
                cv2.polylines(annotated, [pts], True, (0, 255, 0), 3)
                rect = barcode.rect
                cv2.putText(
                    annotated,
                    "QR DETECTED",
                    (rect.left, max(rect.top - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
        return select_unique_payload(payloads), annotated

    def _publish_compressed(self, image: Any, stamp: Any) -> None:
        now = time.time()
        # Publish compressed frame at max ~15 FPS to conserve CPU
        if now - self._last_compress_pub_time < 0.065:
            return
        self._last_compress_pub_time = now

        success, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        try:
            self._compressed_pub.publish(msg)
            self._annotated_compressed_pub.publish(msg)
        except Exception:
            pass

    def _on_image(self, message: Image) -> None:
        self._total_frames += 1
        self._frame_count += 1
        now = time.time()
        if now - self._last_fps_time >= 1.0:
            self._fps = self._frame_count / (now - self._last_fps_time)
            self._frame_count = 0
            self._last_fps_time = now

        try:
            image = image_to_bgr(message, cv2)
            payload, annotated = self._decode_and_annotate(image)
            self._publish_compressed(annotated, message.header.stamp)
        except Exception as exc:
            if now - self._last_heartbeat_time > 2.0:
                print(f"{YELLOW}[图像解析警告] {exc}{RESET}")
                self._last_heartbeat_time = now
            return

        canonical = normalize_payload(payload or "")
        if not canonical:
            if now - self._last_heartbeat_time > 1.5:
                sys.stdout.write(
                    f"\r\033[K{BOLD}[等待中]{RESET} 接收图像流中 ({self._fps:.1f} FPS) - 未检测到二维码图案..."
                )
                sys.stdout.flush()
                self._last_heartbeat_time = now
            return

        # Throttle printing if same payload scanned within 2.0s
        if canonical == self._last_report_payload and (now - self._last_report_time < 2.0):
            return

        self._last_report_payload = canonical
        self._last_report_time = now

        sys.stdout.write("\r\033[K")
        self._report_result(canonical)

    def _report_result(self, raw_text: str) -> None:
        timestamp_str = time.strftime("%H:%M:%S")
        sep_dash = "-" * 70
        print(f"\n{BOLD}{CYAN}{sep_dash}{RESET}")
        print(f"{BOLD}{CYAN}[检测到二维码: {timestamp_str}]{RESET}")
        print(f"{BOLD}【原始解析文本】:{RESET}")
        for idx, line in enumerate(raw_text.split("\n"), start=1):
            print(f"  行 {idx}: {line}")

        # 1. 比赛同款 B 区检验
        b_plan = None
        b_error = None
        try:
            b_plan = build_b_plan(raw_text)
        except ParseError as exc:
            b_error = str(exc)
        except Exception as exc:
            b_error = f"未知错误: {exc}"

        print(f"\n{BOLD}【B 区规则检验】:{RESET}")
        if b_plan is not None:
            print(f"  结果: {GREEN}PASS (合规通过! 任务数={b_plan['task_count']}){RESET}")
            print("  采摘任务顺序:")
            for task in b_plan["tasks"]:
                print(f"    - 果树 {task['position_id']}: {BOLD}{task['target']}{RESET}")
        else:
            print(f"  结果: {RED}FAIL (被比赛流程拒绝并丢弃){RESET}")
            print(f"  原因: {RED}{b_error}{RESET}")

        # 2. 比赛同款 C 区检验
        c_plan = None
        c_error = None
        try:
            c_plan = build_c_plan(raw_text)
        except ParseError as exc:
            c_error = str(exc)
        except Exception as exc:
            c_error = f"未知错误: {exc}"

        print(f"\n{BOLD}【C 区规则检验】:{RESET}")
        if c_plan is not None:
            print(f"  结果: {GREEN}PASS (合规通过! 任务数={c_plan['task_count']}){RESET}")
            print("  前往采摘顺序:")
            for task in c_plan["tasks"]:
                seq = task['sequence']
                pos = task['position_id']
                tgt = task['target']
                print(f"    - 步骤 {seq}: 点位 {BOLD}{pos}{RESET} -> 采摘 {BOLD}{tgt}{RESET}")
        else:
            print(f"  结果: {RED}FAIL (被比赛流程拒绝并丢弃){RESET}")
            print(f"  原因: {RED}{c_error}{RESET}")

        print(f"{BOLD}{CYAN}{sep_dash}{RESET}\n")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = QrTerminalMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print(f"\n{CYAN}[QR TEST] 测试已结束。{RESET}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
