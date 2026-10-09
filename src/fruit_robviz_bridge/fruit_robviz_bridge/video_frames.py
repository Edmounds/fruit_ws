# Copyright (c) 2026 Edmounds
# Copyright (c) 2026 fruit_ws team
#
# Licensed under the MIT License and Apache License, Version 2.0.

"""Stateless image validation and conversion utilities for video streaming."""

from __future__ import annotations

import sys
from typing import Tuple

import cv2
import numpy as np
from sensor_msgs.msg import Image

MAX_FRAME_BYTES = 64 * 1024 * 1024
MAX_DIMENSION = 8192

COLOR_FORMATS = {
    "rgb8": ("RGB", 3),
    "bgr8": ("BGR", 3),
    "rgba8": ("RGBA", 4),
    "bgra8": ("BGRA", 4),
}


def validate_and_extract_color(
    msg: Image,
) -> Tuple[bytes, np.ndarray, Tuple[int, int, str]]:
    """Validate a color Image message and extract GStreamer bytes and BGR mat.

    Returns:
        (gst_bytes, bgr_mat, (width, height, gst_format))
    """
    encoding = msg.encoding.lower()
    format_info = COLOR_FORMATS.get(encoding)
    if not format_info:
        raise ValueError(f"Unsupported color image encoding: {msg.encoding}")

    gst_format, channels = format_info
    width = int(msg.width)
    height = int(msg.height)
    if not (1 <= width <= MAX_DIMENSION and 1 <= height <= MAX_DIMENSION):
        raise ValueError(f"Invalid image dimensions: {width}x{height}")

    row_bytes = width * channels
    gst_stride = (row_bytes + 3) & ~3
    stride = int(msg.step)

    if stride < row_bytes:
        raise ValueError(
            f"Step {stride} is smaller than row bytes {row_bytes}"
        )
    if len(msg.data) < stride * height:
        raise ValueError(
            f"Data length {len(msg.data)} is smaller than expected {stride * height}"
        )
    if stride * height > MAX_FRAME_BYTES:
        raise ValueError(f"Frame size exceeds {MAX_FRAME_BYTES} bytes")

    raw_buffer = memoryview(msg.data)
    if stride == row_bytes:
        contiguous_raw = np.frombuffer(
            raw_buffer[: row_bytes * height], dtype=np.uint8
        ).reshape((height, width, channels))
    else:
        rows = [
            raw_buffer[r * stride: r * stride + row_bytes]
            for r in range(height)
        ]
        contiguous_raw = np.frombuffer(
            b"".join(rows), dtype=np.uint8
        ).reshape((height, width, channels))

    # Produce BGR for OpenCV / JPEG
    if encoding == "bgr8":
        bgr_mat = contiguous_raw
    elif encoding == "rgb8":
        bgr_mat = cv2.cvtColor(contiguous_raw, cv2.COLOR_RGB2BGR)
    elif encoding == "bgra8":
        bgr_mat = cv2.cvtColor(contiguous_raw, cv2.COLOR_BGRA2BGR)
    elif encoding == "rgba8":
        bgr_mat = cv2.cvtColor(contiguous_raw, cv2.COLOR_RGBA2BGR)
    else:
        bgr_mat = contiguous_raw

    # Produce padded bytes for GStreamer appsrc
    if stride == gst_stride:
        gst_bytes = bytes(raw_buffer[: gst_stride * height])
    else:
        pad = bytes(gst_stride - row_bytes)
        gst_bytes = b"".join(
            bytes(raw_buffer[r * stride: r * stride + row_bytes]) + pad
            for r in range(height)
        )

    return gst_bytes, bgr_mat, (width, height, gst_format)


def convert_depth_to_jet(
    msg: Image,
) -> Tuple[bytes, np.ndarray, Tuple[int, int]]:
    """Convert a 16UC1 depth Image to native JET pseudocolor.

    Depth is in millimeters (16UC1). Zero is treated as invalid/missing.
    No downsampling or resizing is performed.

    Returns:
        (gst_rgb_bytes, bgr_mat, (width, height))
    """
    if msg.encoding != "16UC1":
        raise ValueError(
            f"Native depth preview requires 16UC1 encoding, got {msg.encoding}"
        )

    width = int(msg.width)
    height = int(msg.height)
    if not (1 <= width <= MAX_DIMENSION and 1 <= height <= MAX_DIMENSION):
        raise ValueError(f"Invalid depth image dimensions: {width}x{height}")

    row_bytes = width * 2
    stride = int(msg.step)
    if stride < row_bytes:
        raise ValueError(
            f"Depth step {stride} is smaller than row bytes {row_bytes}"
        )
    if len(msg.data) < stride * height:
        raise ValueError(
            f"Depth data length {len(msg.data)} is smaller than expected {stride * height}"
        )
    if stride * height > MAX_FRAME_BYTES:
        raise ValueError(f"Depth frame exceeds {MAX_FRAME_BYTES} bytes")

    raw_buffer = memoryview(msg.data)
    dtype = np.dtype(np.uint16)
    if msg.is_bigendian and sys.byteorder != "big":
        dtype = dtype.newbyteorder(">")
    elif not msg.is_bigendian and sys.byteorder != "little":
        dtype = dtype.newbyteorder("<")

    if stride == row_bytes:
        depth_array = np.frombuffer(
            raw_buffer[: row_bytes * height], dtype=dtype
        ).reshape((height, width))
    else:
        rows = [
            raw_buffer[r * stride: r * stride + row_bytes]
            for r in range(height)
        ]
        depth_array = np.frombuffer(b"".join(rows), dtype=dtype).reshape(
            (height, width)
        )

    if depth_array.dtype.byteorder not in ("=", "|"):
        depth_array = depth_array.astype(np.uint16)

    valid_mask = depth_array > 0
    valid_count = int(np.count_nonzero(valid_mask))
    total_pixels = width * height

    colored_bgr = np.zeros((height, width, 3), dtype=np.uint8)
    min_m = 0.0
    max_m = 0.0

    if valid_count > 0:
        valid_values = depth_array[valid_mask]
        min_val = float(np.min(valid_values))
        max_val = float(np.max(valid_values))
        min_m = min_val * 0.001
        max_m = max_val * 0.001

        normalized = np.zeros((height, width), dtype=np.uint8)
        if min_val == max_val:
            normalized[valid_mask] = 128
        else:
            scale = 254.0 / (max_val - min_val)
            normalized_vals = (valid_values - min_val) * scale + 1.0
            normalized[valid_mask] = np.clip(normalized_vals, 1, 255).astype(
                np.uint8
            )

        mapped_bgr = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
        colored_bgr[valid_mask] = mapped_bgr[valid_mask]

    # Banner overlay: 22px height black bar at top
    banner_text = (
        f"Depth ({width}x{height}) | "
        f"{min_m:.2f}~{max_m:.2f}m | {100.0 * valid_count / total_pixels:.0f}%"
        if valid_count > 0
        else f"Depth ({width}x{height}) | No valid depth"
    )

    cv2.rectangle(
        colored_bgr, (0, 0), (width, min(height, 22)), (20, 20, 20), cv2.FILLED
    )
    cv2.putText(
        colored_bgr,
        banner_text,
        (5, 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (0, 255, 120),
        1,
        cv2.LINE_AA,
    )

    # Convert to RGB bytes for GStreamer
    rgb_mat = cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGB)
    row_bytes_rgb = width * 3
    gst_stride = (row_bytes_rgb + 3) & ~3
    if gst_stride == row_bytes_rgb:
        gst_bytes = rgb_mat.tobytes()
    else:
        pad = bytes(gst_stride - row_bytes_rgb)
        gst_bytes = b"".join(
            rgb_mat[r, :].tobytes() + pad for r in range(height)
        )

    return gst_bytes, colored_bgr, (width, height)


def encode_jpeg(bgr_mat: np.ndarray, quality: int) -> bytes:
    """Encode BGR image to JPEG bytes with specified quality (30..95)."""
    q = max(30, min(95, int(quality)))
    success, buffer = cv2.imencode(
        ".jpg", bgr_mat, [int(cv2.IMWRITE_JPEG_QUALITY), q]
    )
    if not success:
        raise RuntimeError("OpenCV JPEG encoding failed")
    return buffer.tobytes()


def encode_png(bgr_mat: np.ndarray, compression_level: int = 3) -> bytes:
    """Encode BGR image to PNG bytes losslessly."""
    level = max(0, min(9, int(compression_level)))
    success, buffer = cv2.imencode(
        ".png", bgr_mat, [int(cv2.IMWRITE_PNG_COMPRESSION), level]
    )
    if not success:
        raise RuntimeError("OpenCV PNG encoding failed")
    return buffer.tobytes()
