#!/usr/bin/env python3
"""Direct serial console for the fruit robot lower controller.

This utility speaks the wire protocol directly. It does not start ROS or
import auto_serial_bridge; the protocol YAML is read only for configuration.
"""

from __future__ import annotations

import argparse
import math
import shlex
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Deque, Dict, Iterable, List, Optional, TextIO, Tuple

import yaml

try:
    import serial
except ImportError:  # pragma: no cover - exercised by the CLI environment.
    serial = None


DEFAULT_PROTOCOL_PATH = (
    Path(__file__).resolve().parent
    / "src"
    / "auto_serial_bridge"
    / "config"
    / "protocol.yaml"
)

PACKET_ACK = 0xFD
PACKET_HEARTBEAT = 0xFE
PACKET_HANDSHAKE = 0xFF

ANSI_CODES = {
    "blue": "\033[34m",
    "cyan": "\033[36m",
    "green": "\033[32m",
    "yellow": "\033[33m",
    "red": "\033[31m",
    "dim": "\033[2m",
}
ANSI_RESET = "\033[0m"

TASK_ALIASES = {
    "grip": (0, 0),
    "open": (0, 1),
    "gimbal-down": (1, 0),
    "gimbal-up": (1, 1),
    "left": (2, 0),
    "right": (2, 1),
    "qr-pose": (4, 0),
}
AREA_STATUS = {"A": 0, "B": 1, "C": 2, "D": 3}
CALLBACK_NAMES = {
    0: "gripper",
    1: "camera gimbal",
    2: "side observation position",
    3: "arm target",
    4: "QR recognition pose",
    5: "current area",
}
CALLBACK_STATUS_NAMES = {0: "completed", 1: "executing"}
TASK_NAMES = {
    0: "gripper",
    1: "camera gimbal",
    2: "side observation position",
    4: "QR recognition pose",
    5: "current area",
}


class ProtocolError(ValueError):
    """Raised when the protocol configuration or a frame is invalid."""


class CommandError(ValueError):
    """Raised when an interactive command has invalid arguments."""


@dataclass(frozen=True)
class MessageSpec:
    name: str
    packet_id: int
    direction: str
    reliable: bool


@dataclass(frozen=True)
class ProtocolConfig:
    port: str
    baudrate: int
    head_byte_1: int
    head_byte_2: int
    checksum: str
    require_handshake: bool
    strict_heartbeat: bool
    heartbeat_interval_s: float
    heartbeat_timeout_s: float
    reliable_retry_interval_s: float
    reliable_retry_warning: int
    messages: Dict[str, MessageSpec]
    source_path: Optional[Path] = None

    @classmethod
    def from_mapping(
        cls, mapping: dict, source_path: Optional[Path] = None
    ) -> "ProtocolConfig":
        if not isinstance(mapping, dict):
            raise ProtocolError("protocol YAML root must be a mapping")

        serial_section = mapping.get("serial_controller", {})
        parameters = serial_section.get("ros__parameters", {})
        config = mapping.get("config", {})
        raw_messages = mapping.get("messages", [])
        if not isinstance(parameters, dict) or not isinstance(config, dict):
            raise ProtocolError("protocol YAML has invalid serial_controller/config sections")
        if not isinstance(raw_messages, list):
            raise ProtocolError("protocol YAML messages must be a list")

        messages: Dict[str, MessageSpec] = {}
        used_ids = set()
        for raw_message in raw_messages:
            if not isinstance(raw_message, dict):
                raise ProtocolError("each protocol message must be a mapping")
            name = str(raw_message.get("name", ""))
            if not name or name in messages:
                raise ProtocolError(f"invalid or duplicate protocol message name: {name!r}")
            packet_id = _parse_config_int(raw_message.get("id"), f"message {name} id")
            if not 0 <= packet_id <= 0xFC:
                raise ProtocolError(f"message {name} id must be in [0, 0xfc]")
            if packet_id in used_ids:
                raise ProtocolError(f"duplicate protocol packet id: 0x{packet_id:02x}")
            used_ids.add(packet_id)
            messages[name] = MessageSpec(
                name=name,
                packet_id=packet_id,
                direction=str(raw_message.get("direction", "both")),
                reliable=bool(raw_message.get("reliable", False)),
            )

        required_messages = {
            "StateMachineCommand",
            "ExecutionCallback",
            "ArmTarget",
            "VelocityCommand",
        }
        missing = required_messages.difference(messages)
        if missing:
            raise ProtocolError(
                "protocol YAML is missing required messages: "
                + ", ".join(sorted(missing))
            )

        checksum = str(config.get("checksum", "CRC8")).upper()
        if checksum not in {"NONE", "SUM8", "XOR8", "CRC8"}:
            raise ProtocolError(f"unsupported checksum algorithm: {checksum}")

        heartbeat_interval_ms = _parse_config_int(
            config.get("heartbeat_interval_ms"), "heartbeat_interval_ms"
        )
        heartbeat_timeout_ms = _parse_config_int(
            config.get("heartbeat_timeout_ms"), "heartbeat_timeout_ms"
        )
        retry_interval_ms = _parse_config_int(
            config.get("reliable_retry_interval_ms"),
            "reliable_retry_interval_ms",
        )
        retry_warning = _parse_config_int(
            config.get("reliable_max_retries"), "reliable_max_retries"
        )
        if heartbeat_interval_ms <= 0 or heartbeat_timeout_ms <= 0:
            raise ProtocolError("heartbeat intervals must be positive")
        if retry_interval_ms <= 0:
            raise ProtocolError("reliable_retry_interval_ms must be positive")
        if retry_warning < 0:
            raise ProtocolError("reliable_max_retries cannot be negative")

        return cls(
            port=str(parameters.get("port", "/dev/stm32")),
            baudrate=_parse_config_int(parameters.get("baudrate", 115200), "baudrate"),
            head_byte_1=_parse_config_int(config.get("head_byte_1"), "head_byte_1"),
            head_byte_2=_parse_config_int(config.get("head_byte_2"), "head_byte_2"),
            checksum=checksum,
            require_handshake=bool(config.get("require_handshake", False)),
            strict_heartbeat=bool(config.get("strict_heartbeat", False)),
            heartbeat_interval_s=heartbeat_interval_ms / 1000.0,
            heartbeat_timeout_s=heartbeat_timeout_ms / 1000.0,
            reliable_retry_interval_s=retry_interval_ms / 1000.0,
            reliable_retry_warning=retry_warning,
            messages=messages,
            source_path=source_path,
        )

    def message(self, name: str) -> MessageSpec:
        try:
            return self.messages[name]
        except KeyError as exc:
            raise ProtocolError(f"message is not defined in protocol: {name}") from exc

    def message_by_id(self, packet_id: int) -> Optional[MessageSpec]:
        for message in self.messages.values():
            if message.packet_id == packet_id:
                return message
        system_messages = {
            PACKET_ACK: MessageSpec("Ack", PACKET_ACK, "both", False),
            PACKET_HEARTBEAT: MessageSpec("Heartbeat", PACKET_HEARTBEAT, "both", False),
            PACKET_HANDSHAKE: MessageSpec("Handshake", PACKET_HANDSHAKE, "both", False),
        }
        return system_messages.get(packet_id)


def _parse_config_int(value: object, label: str) -> int:
    if value is None:
        raise ProtocolError(f"missing protocol value: {label}")
    try:
        if isinstance(value, str):
            try:
                return int(value, 0)
            except ValueError:
                return int(value, 10)
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"invalid integer for {label}: {value!r}") from exc


def load_protocol(path: Path) -> ProtocolConfig:
    try:
        with path.open("r", encoding="utf-8") as protocol_file:
            mapping = yaml.safe_load(protocol_file)
    except OSError as exc:
        raise ProtocolError(f"cannot read protocol file {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ProtocolError(f"cannot parse protocol file {path}: {exc}") from exc
    return ProtocolConfig.from_mapping(mapping, source_path=path)


def resolve_connection_options(
    protocol: ProtocolConfig,
    port_override: Optional[str] = None,
    baudrate_override: Optional[int] = None,
) -> Tuple[str, int]:
    port = port_override if port_override else protocol.port
    baudrate = baudrate_override if baudrate_override is not None else protocol.baudrate
    return port, baudrate


def calculate_checksum(data: bytes, algorithm: str) -> int:
    algorithm = algorithm.upper()
    if algorithm == "NONE":
        return 0
    if algorithm == "SUM8":
        return sum(data) & 0xFF
    if algorithm == "XOR8":
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    if algorithm == "CRC8":
        checksum = 0
        for byte in data:
            checksum ^= byte
            for _ in range(8):
                if checksum & 0x80:
                    checksum = ((checksum << 1) ^ 0x31) & 0xFF
                else:
                    checksum = (checksum << 1) & 0xFF
        return checksum
    raise ProtocolError(f"unsupported checksum algorithm: {algorithm}")


@dataclass(frozen=True)
class Frame:
    packet_id: int
    payload: bytes
    raw: bytes


def build_frame(protocol: ProtocolConfig, packet_id: int, payload: bytes) -> bytes:
    if not 0 <= packet_id <= 0xFF:
        raise ProtocolError(f"packet id out of range: {packet_id}")
    if len(payload) > 0xFF:
        raise ProtocolError(f"payload is too large: {len(payload)} bytes")
    body = bytes((packet_id, len(payload))) + payload
    checksum = calculate_checksum(body, protocol.checksum)
    return bytes((protocol.head_byte_1, protocol.head_byte_2)) + body + bytes((checksum,))


class FrameParser:
    """Incremental parser that resynchronizes on the configured frame header."""

    def __init__(self, protocol: ProtocolConfig):
        self.protocol = protocol
        self._buffer = bytearray()
        self._errors: Deque[str] = deque()
        self._header = bytes((protocol.head_byte_1, protocol.head_byte_2))

    def feed(self, data: bytes) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []

        while True:
            header_index = self._buffer.find(self._header)
            if header_index < 0:
                if self._buffer and self._buffer[-1] == self.protocol.head_byte_1:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if header_index:
                del self._buffer[:header_index]
            if len(self._buffer) < 4:
                break

            payload_length = self._buffer[3]
            frame_length = 2 + 1 + 1 + payload_length + 1
            if len(self._buffer) < frame_length:
                break

            raw = bytes(self._buffer[:frame_length])
            del self._buffer[:frame_length]
            expected_checksum = calculate_checksum(raw[2:-1], self.protocol.checksum)
            if raw[-1] != expected_checksum:
                self._errors.append(
                    "checksum mismatch: "
                    f"packet_id=0x{raw[2]:02x}, expected=0x{expected_checksum:02x}, "
                    f"got=0x{raw[-1]:02x}"
                )
                continue
            frames.append(Frame(packet_id=raw[2], payload=raw[4:-1], raw=raw))

        return frames

    def pop_errors(self) -> List[str]:
        errors = list(self._errors)
        self._errors.clear()
        return errors


@dataclass(frozen=True)
class DecodedFrame:
    kind: str
    text: str
    color: str
    fields: Dict[str, object]


def _malformed_frame(name: str, payload: bytes, expected: int) -> DecodedFrame:
    return DecodedFrame(
        kind="error",
        text=(
            f"RX {name}: malformed payload length={len(payload)}, "
            f"expected={expected}, raw={payload.hex(' ')}"
        ),
        color="red",
        fields={},
    )


def decode_frame(protocol: ProtocolConfig, frame: Frame) -> DecodedFrame:
    spec = protocol.message_by_id(frame.packet_id)
    name = spec.name if spec else f"packet 0x{frame.packet_id:02x}"

    if frame.packet_id == PACKET_ACK:
        if len(frame.payload) != 2:
            return _malformed_frame(name, frame.payload, 2)
        acked_id, ack_seq = frame.payload
        return DecodedFrame(
            kind="ack",
            text=f"RX Ack: packet_id=0x{acked_id:02x}, seq={ack_seq}",
            color="cyan",
            fields={"acked_id": acked_id, "ack_seq": ack_seq},
        )

    if frame.packet_id == PACKET_HEARTBEAT:
        if len(frame.payload) != 4:
            return _malformed_frame(name, frame.payload, 4)
        count = struct.unpack("<I", frame.payload)[0]
        return DecodedFrame(
            kind="heartbeat",
            text=f"RX Heartbeat: count={count}",
            color="cyan",
            fields={"count": count},
        )

    if frame.packet_id == PACKET_HANDSHAKE:
        if len(frame.payload) != 4:
            return _malformed_frame(name, frame.payload, 4)
        protocol_hash = struct.unpack("<I", frame.payload)[0]
        return DecodedFrame(
            kind="handshake",
            text=f"RX Handshake: protocol_hash=0x{protocol_hash:08x}",
            color="cyan",
            fields={"protocol_hash": protocol_hash},
        )

    callback_id = protocol.message("ExecutionCallback").packet_id
    if frame.packet_id == callback_id:
        if len(frame.payload) != 2:
            return _malformed_frame(name, frame.payload, 2)
        callback_value, status_value = frame.payload
        callback_name = CALLBACK_NAMES.get(callback_value, "unknown")
        status_name = CALLBACK_STATUS_NAMES.get(status_value, "unknown")
        color = "green" if status_value == 0 else "yellow" if status_value == 1 else "red"
        return DecodedFrame(
            kind="callback",
            text=(
                f"RX ExecutionCallback: callback={callback_name}({callback_value}), "
                f"status={status_name}({status_value})"
            ),
            color=color,
            fields={"callback_id": callback_value, "callback_status": status_value},
        )

    state_id = protocol.message("StateMachineCommand").packet_id
    if frame.packet_id == state_id:
        expected = 3 if protocol.message("StateMachineCommand").reliable else 2
        if len(frame.payload) != expected:
            return _malformed_frame(name, frame.payload, expected)
        task_id, task_status = frame.payload[:2]
        fields: Dict[str, object] = {"task_id": task_id, "task_status": task_status}
        suffix = ""
        if expected == 3:
            fields["sequence"] = frame.payload[2]
            suffix = f", seq={frame.payload[2]}"
        task_name = TASK_NAMES.get(task_id, "unknown")
        return DecodedFrame(
            kind="state",
            text=(
                f"RX StateMachineCommand: task={task_name}({task_id}), "
                f"status={task_status}{suffix}"
            ),
            color="yellow",
            fields=fields,
        )

    arm_id = protocol.message("ArmTarget").packet_id
    if frame.packet_id == arm_id:
        expected = 14 if protocol.message("ArmTarget").reliable else 13
        if len(frame.payload) != expected:
            return _malformed_frame(name, frame.payload, expected)
        target_x, target_y, target_z, z_type = struct.unpack("<fffB", frame.payload[:13])
        fields = {
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "z_type": z_type,
        }
        suffix = ""
        if expected == 14:
            fields["sequence"] = frame.payload[13]
            suffix = f", seq={frame.payload[13]}"
        return DecodedFrame(
            kind="arm_target",
            text=(
                "RX ArmTarget: "
                f"x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}, "
                f"z_type={z_type}{suffix}"
            ),
            color="yellow",
            fields=fields,
        )

    velocity_id = protocol.message("VelocityCommand").packet_id
    if frame.packet_id == velocity_id:
        if len(frame.payload) != 8:
            return _malformed_frame(name, frame.payload, 8)
        linear_x, angular_z = struct.unpack("<ff", frame.payload)
        return DecodedFrame(
            kind="velocity",
            text=f"RX VelocityCommand: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}",
            color="yellow",
            fields={"linear_x": linear_x, "angular_z": angular_z},
        )

    return DecodedFrame(
        kind="unknown",
        text=(
            f"RX Unknown packet_id=0x{frame.packet_id:02x}, "
            f"payload={frame.payload.hex(' ')}"
        ),
        color="red",
        fields={},
    )


@dataclass(frozen=True)
class OutgoingMessage:
    name: str
    payload: bytes


@dataclass(frozen=True)
class ParsedCommand:
    action: str
    message: Optional[OutgoingMessage] = None


def _parse_user_int(value: str, label: str, minimum: int, maximum: int) -> int:
    try:
        try:
            parsed = int(value, 0)
        except ValueError:
            parsed = int(value, 10)
    except ValueError as exc:
        raise CommandError(f"{label} must be an integer: {value}") from exc
    if not minimum <= parsed <= maximum:
        raise CommandError(f"{label} must be in [{minimum}, {maximum}]")
    return parsed


def _parse_user_float(value: str, label: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise CommandError(f"{label} must be a number: {value}") from exc
    if not math.isfinite(parsed):
        raise CommandError(f"{label} must be finite")
    return parsed


def _check_arg_count(tokens: List[str], expected: int, usage: str) -> None:
    if len(tokens) != expected:
        raise CommandError(f"usage: {usage}")


def _state_message(protocol: ProtocolConfig, task_id: int, task_status: int) -> OutgoingMessage:
    protocol.message("StateMachineCommand")
    return OutgoingMessage(
        name="StateMachineCommand",
        payload=bytes((task_id, task_status)),
    )


def parse_command(line: str, protocol: ProtocolConfig) -> ParsedCommand:
    try:
        tokens = shlex.split(line)
    except ValueError as exc:
        raise CommandError(f"cannot parse command: {exc}") from exc
    if not tokens:
        return ParsedCommand(action="empty")

    command = tokens[0].lower()
    if command in {"help", "?"}:
        _check_arg_count(tokens, 1, "help")
        return ParsedCommand(action="help")
    if command in {"quit", "exit"}:
        _check_arg_count(tokens, 1, "quit")
        return ParsedCommand(action="quit")
    if command in TASK_ALIASES:
        _check_arg_count(tokens, 1, command)
        task_id, task_status = TASK_ALIASES[command]
        return ParsedCommand(action="send", message=_state_message(protocol, task_id, task_status))
    if command == "task":
        _check_arg_count(tokens, 3, "task <task_id> <task_status>")
        task_id = _parse_user_int(tokens[1], "task_id", 0, 0xFF)
        task_status = _parse_user_int(tokens[2], "task_status", 0, 0xFF)
        return ParsedCommand(action="send", message=_state_message(protocol, task_id, task_status))
    if command == "area":
        _check_arg_count(tokens, 2, "area <A|B|C|D>")
        try:
            status = AREA_STATUS[tokens[1].upper()]
        except KeyError as exc:
            raise CommandError("area must be one of A, B, C, or D") from exc
        return ParsedCommand(action="send", message=_state_message(protocol, 5, status))
    if command == "arm":
        _check_arg_count(tokens, 5, "arm <x> <y> <z> <z_type>")
        target_x = _parse_user_float(tokens[1], "target_x")
        target_y = _parse_user_float(tokens[2], "target_y")
        target_z = _parse_user_float(tokens[3], "target_z")
        z_type = _parse_user_int(tokens[4], "z_type", 0, 15)
        protocol.message("ArmTarget")
        return ParsedCommand(
            action="send",
            message=OutgoingMessage(
                name="ArmTarget",
                payload=struct.pack("<fffB", target_x, target_y, target_z, z_type),
            ),
        )
    if command in {"vel", "velocity"}:
        _check_arg_count(tokens, 3, "vel <linear_x> <angular_z>")
        linear_x = _parse_user_float(tokens[1], "linear_x")
        angular_z = _parse_user_float(tokens[2], "angular_z")
        protocol.message("VelocityCommand")
        return ParsedCommand(
            action="send",
            message=OutgoingMessage(
                name="VelocityCommand",
                payload=struct.pack("<ff", linear_x, -angular_z),
            ),
        )
    if command == "stop":
        _check_arg_count(tokens, 1, "stop")
        protocol.message("VelocityCommand")
        return ParsedCommand(
            action="send",
            message=OutgoingMessage(
                name="VelocityCommand",
                payload=struct.pack("<ff", 0.0, 0.0),
            ),
        )

    raise CommandError(f"unknown command: {tokens[0]}")


def encode_outgoing(
    protocol: ProtocolConfig,
    message: OutgoingMessage,
    sequence: int = 0,
) -> Tuple[bytes, Optional[int]]:
    spec = protocol.message(message.name)
    if spec.reliable:
        if not 0 <= sequence <= 0xFF:
            raise ProtocolError(f"reliable sequence out of range: {sequence}")
        payload = message.payload + bytes((sequence,))
        return build_frame(protocol, spec.packet_id, payload), sequence
    return build_frame(protocol, spec.packet_id, message.payload), None


@dataclass
class PendingReliable:
    message: OutgoingMessage
    packet_id: int
    sequence: int
    frame: bytes
    sent_at: float = 0.0
    retry_count: int = 0


class ReliableQueue:
    """FIFO reliable sender using one sequence-preserving frame per command."""

    def __init__(
        self,
        protocol: ProtocolConfig,
        send: Callable[[PendingReliable, bool], None],
        on_warning: Optional[Callable[[PendingReliable], None]] = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.protocol = protocol
        self._send = send
        self._on_warning = on_warning
        self._clock = clock
        self._queue: Deque[PendingReliable] = deque()
        self._next_sequence = 0
        self._lock = threading.RLock()

    def enqueue(self, message: OutgoingMessage) -> PendingReliable:
        spec = self.protocol.message(message.name)
        if not spec.reliable:
            raise ProtocolError(f"message is not reliable: {message.name}")
        frame, sequence = encode_outgoing(self.protocol, message, self._next_sequence)
        assert sequence is not None
        self._next_sequence = (self._next_sequence + 1) & 0xFF
        entry = PendingReliable(message, spec.packet_id, sequence, frame)
        with self._lock:
            self._queue.append(entry)
            if len(self._queue) == 1:
                self._transmit_head(retry=False)
        return entry

    def _transmit_head(self, retry: bool) -> None:
        entry = self._queue[0]
        entry.sent_at = self._clock()
        if retry:
            entry.retry_count += 1
        self._send(entry, retry)
        if (
            retry
            and self.protocol.reliable_retry_warning > 0
            and entry.retry_count % self.protocol.reliable_retry_warning == 0
            and self._on_warning is not None
        ):
            self._on_warning(entry)

    def tick(self, now: Optional[float] = None) -> None:
        with self._lock:
            if not self._queue:
                return
            current = self._clock() if now is None else now
            if current - self._queue[0].sent_at < self.protocol.reliable_retry_interval_s:
                return
            self._transmit_head(retry=True)

    def handle_ack(self, packet_id: int, sequence: int) -> bool:
        with self._lock:
            if not self._queue:
                return False
            entry = self._queue[0]
            if entry.packet_id != packet_id or entry.sequence != sequence:
                return False
            self._queue.popleft()
            if self._queue:
                self._transmit_head(retry=False)
            return True

    def __len__(self) -> int:
        with self._lock:
            return len(self._queue)


@dataclass
class HeartbeatState:
    count: int
    started_at: float
    last_sent_at: float


class HeartbeatManager:
    def __init__(
        self,
        protocol: ProtocolConfig,
        send: Callable[[int, bool], None],
        on_timeout: Callable[[int], None],
        clock: Callable[[], float] = time.monotonic,
    ):
        self.protocol = protocol
        self._send = send
        self._on_timeout = on_timeout
        self._clock = clock
        self._next_count = 0
        self._state: Optional[HeartbeatState] = None
        self._next_start_at: Optional[float] = None
        self._lock = threading.Lock()

    def start(self, now: Optional[float] = None) -> None:
        with self._lock:
            current = self._clock() if now is None else now
            self._state = HeartbeatState(self._next_count, current, current)
            self._next_start_at = None
            self._next_count = (self._next_count + 1) & 0xFFFFFFFF
            self._send(self._state.count, False)

    def _start_next(self, now: float) -> None:
        self._state = HeartbeatState(self._next_count, now, now)
        self._next_start_at = None
        self._next_count = (self._next_count + 1) & 0xFFFFFFFF
        self._send(self._state.count, False)

    def tick(self, now: Optional[float] = None) -> None:
        with self._lock:
            current = self._clock() if now is None else now
            if self._state is None:
                if self._next_start_at is None or current >= self._next_start_at:
                    self._start_next(current)
                return
            if current - self._state.started_at > self.protocol.heartbeat_timeout_s:
                timed_out_count = self._state.count
                self._on_timeout(timed_out_count)
                if self.protocol.strict_heartbeat:
                    self._state = None
                    self._next_start_at = None
                    return
                self._start_next(current)
                return
            if current - self._state.last_sent_at >= self.protocol.heartbeat_interval_s:
                self._state.last_sent_at = current
                self._send(self._state.count, True)

    def handle_echo(self, count: int) -> bool:
        with self._lock:
            if self._state is None or self._state.count != count:
                return False
            self._state = None
            self._next_start_at = self._clock() + self.protocol.heartbeat_interval_s
            return True


def colorize(text: str, color: str, enabled: bool) -> str:
    if not enabled or color not in ANSI_CODES:
        return text
    return f"{ANSI_CODES[color]}{text}{ANSI_RESET}"


class ConsoleOutput:
    def __init__(self, no_color: bool = False, stream: Optional[TextIO] = None):
        self.stream = stream or sys.stdout
        self.enabled = not no_color and bool(getattr(self.stream, "isatty", lambda: False)())
        self._lock = threading.Lock()

    def line(self, text: str, color: str = "dim") -> None:
        with self._lock:
            print(colorize(text, color, self.enabled), file=self.stream, flush=True)


class SerialSession:
    def __init__(
        self,
        serial_port,
        protocol: ProtocolConfig,
        output: ConsoleOutput,
        show_heartbeat: bool = False,
    ):
        self.serial_port = serial_port
        self.protocol = protocol
        self.output = output
        self.show_heartbeat = show_heartbeat
        self.stop_event = threading.Event()
        self.write_lock = threading.Lock()
        self.parser = FrameParser(protocol)
        self.reliable = ReliableQueue(
            protocol,
            self._send_reliable,
            on_warning=self._on_reliable_warning,
        )
        self.heartbeat = HeartbeatManager(
            protocol,
            self._send_heartbeat,
            on_timeout=self._on_heartbeat_timeout,
        )
        self._threads: List[threading.Thread] = []

    def start(self) -> None:
        self.heartbeat.start()
        self._threads = [
            threading.Thread(target=self._reader_loop, name="lower-controller-rx", daemon=True),
            threading.Thread(target=self._timer_loop, name="lower-controller-timer", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def close(self) -> None:
        self.stop_event.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        try:
            if self.serial_port.is_open:
                self.serial_port.close()
        except (AttributeError, OSError):
            pass

    def send_message(self, message: OutgoingMessage) -> None:
        if self.protocol.message(message.name).reliable:
            self.reliable.enqueue(message)
            return
        frame, _ = encode_outgoing(self.protocol, message)
        self._write_frame(frame, f"TX {message.name} ({len(frame)} bytes)", "blue")

    def _write_frame(
        self,
        frame: bytes,
        description: str,
        color: str,
        show_output: bool = True,
    ) -> None:
        try:
            with self.write_lock:
                written = self.serial_port.write(frame)
                if written != len(frame):
                    raise OSError(f"short serial write: {written}/{len(frame)} bytes")
                self.serial_port.flush()
        except Exception as exc:  # serial backends expose different exception types.
            self.output.line(f"Serial write failed: {exc}", "red")
            self.stop_event.set()
            return
        if show_output:
            self.output.line(f"{description}: {frame.hex(' ')}", color)

    def _send_reliable(self, entry: PendingReliable, retry: bool) -> None:
        action = "retry" if retry else "send"
        self._write_frame(
            entry.frame,
            f"TX {action} {entry.message.name} seq={entry.sequence}",
            "yellow" if retry else "blue",
        )

    def _send_heartbeat(self, count: int, retry: bool) -> None:
        frame = build_frame(self.protocol, PACKET_HEARTBEAT, struct.pack("<I", count))
        self._write_frame(
            frame,
            f"TX {'heartbeat retry' if retry else 'heartbeat'} count={count}",
            "yellow" if retry else "blue",
            show_output=self.show_heartbeat,
        )

    def _on_reliable_warning(self, entry: PendingReliable) -> None:
        self.output.line(
            f"Reliable packet still awaiting ACK: {entry.message.name} "
            f"seq={entry.sequence}, retries={entry.retry_count}",
            "yellow",
        )

    def _on_heartbeat_timeout(self, count: int) -> None:
        if self.show_heartbeat:
            self.output.line(f"Heartbeat timeout: count={count}", "red")
        if self.protocol.strict_heartbeat:
            self.stop_event.set()

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self.serial_port.read(256)
            except Exception as exc:  # serial backends expose different exception types.
                self.output.line(f"Serial read failed: {exc}", "red")
                self.stop_event.set()
                return
            if not data:
                continue
            for error in self.parser.pop_errors():
                self.output.line(f"RX frame error: {error}", "red")
            for frame in self.parser.feed(data):
                self._handle_frame(frame)
            for error in self.parser.pop_errors():
                self.output.line(f"RX frame error: {error}", "red")

    def _handle_frame(self, frame: Frame) -> None:
        decoded = decode_frame(self.protocol, frame)
        if decoded.kind != "heartbeat" or self.show_heartbeat:
            self.output.line(decoded.text, decoded.color)
        if decoded.kind == "ack":
            matched = self.reliable.handle_ack(
                int(decoded.fields["acked_id"]),
                int(decoded.fields["ack_seq"]),
            )
            if matched:
                self.output.line("Reliable command ACK matched", "cyan")
            else:
                self.output.line("ACK did not match the pending reliable command", "yellow")
        elif decoded.kind == "heartbeat":
            if not self.heartbeat.handle_echo(int(decoded.fields["count"])):
                self.output.line("Heartbeat count did not match the pending heartbeat", "red")

    def _timer_loop(self) -> None:
        while not self.stop_event.wait(0.02):
            try:
                self.reliable.tick()
                self.heartbeat.tick()
            except Exception as exc:
                self.output.line(f"Background serial task failed: {exc}", "red")
                self.stop_event.set()
                return


def help_text() -> str:
    return """Commands:
  grip                         gripper grip
  open                         gripper open
  gimbal-down | gimbal-up     camera gimbal position
  left | right                 side observation position
  qr-pose                      QR recognition pose
  area A|B|C|D                set current area
  task <id> <status>           send a raw StateMachineCommand
  arm <x> <y> <z> <z_type>     send an arm target in meters
  vel <linear_x> <angular_z>   send one velocity command
  stop                         send zero velocity
  help                         show this help
  quit                         close the serial port and exit"""


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Direct interactive serial console for the fruit robot lower controller"
    )
    parser.add_argument(
        "--protocol",
        default=str(DEFAULT_PROTOCOL_PATH),
        help="protocol YAML path (default: %(default)s)",
    )
    parser.add_argument("--port", help="override the serial device from protocol YAML")
    parser.add_argument(
        "--baudrate",
        type=int,
        help="override the baudrate from protocol YAML",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="disable ANSI colors",
    )
    parser.add_argument(
        "--show-heartbeat",
        action="store_true",
        help="show heartbeat TX/RX and heartbeat timeout messages",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if serial is None:
        print(
            "python3-serial is required; install the system package python3-serial",
            file=sys.stderr,
        )
        return 2

    try:
        protocol = load_protocol(Path(args.protocol).expanduser())
    except ProtocolError as exc:
        print(f"Protocol configuration error: {exc}", file=sys.stderr)
        return 2
    if protocol.require_handshake:
        print(
            "This console expects require_handshake=false in the current protocol.",
            file=sys.stderr,
        )
        return 2

    port, baudrate = resolve_connection_options(protocol, args.port, args.baudrate)
    output = ConsoleOutput(no_color=args.no_color)
    try:
        serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0.1,
            write_timeout=1.0,
        )
    except Exception as exc:  # serial.SerialException differs across pyserial versions.
        output.line(f"Cannot open serial port {port} at {baudrate}: {exc}", "red")
        return 1

    output.line(
        f"Connected directly to {port} at {baudrate}; auto_serial_bridge is not used.",
        "green",
    )
    output.line(
        "Automatic heartbeat is enabled but hidden; reliable retries are enabled. "
        "Type help for commands.",
        "cyan",
    )
    session = SerialSession(
        serial_port,
        protocol,
        output,
        show_heartbeat=args.show_heartbeat,
    )
    session.start()
    try:
        while not session.stop_event.is_set():
            try:
                line = input("fruit> ")
            except EOFError:
                break
            except KeyboardInterrupt:
                output.line("", "dim")
                break
            try:
                command = parse_command(line, protocol)
                if command.action == "empty":
                    continue
                if command.action == "help":
                    output.line(help_text(), "dim")
                    continue
                if command.action == "quit":
                    break
                assert command.message is not None
                session.send_message(command.message)
            except (CommandError, ProtocolError) as exc:
                output.line(f"Command error: {exc}", "red")
    finally:
        session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
