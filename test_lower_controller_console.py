import struct

import yaml

from lower_controller_console import (
    FrameParser,
    HeartbeatManager,
    ProtocolConfig,
    ReliableQueue,
    build_frame,
    calculate_checksum,
    colorize,
    decode_frame,
    encode_outgoing,
    load_protocol,
    parse_command,
    resolve_connection_options,
)


def protocol_mapping(**config_overrides):
    config = {
        "buffer_size": 1024,
        "head_byte_1": 0x5A,
        "head_byte_2": 0xA5,
        "checksum": "CRC8",
        "require_handshake": False,
        "heartbeat_interval_ms": 1000,
        "heartbeat_timeout_ms": 3000,
        "reliable_retry_interval_ms": 100,
        "reliable_max_retries": 3,
    }
    config.update(config_overrides)
    return {
        "serial_controller": {
            "ros__parameters": {
                "port": "/dev/test-stm32",
                "baudrate": 230400,
            }
        },
        "config": config,
        "messages": [
            {
                "name": "StateMachineCommand",
                "id": 0x11,
                "direction": "tx",
                "reliable": True,
            },
            {
                "name": "ExecutionCallback",
                "id": 0x12,
                "direction": "rx",
            },
            {
                "name": "ArmTarget",
                "id": 0x13,
                "direction": "tx",
                "reliable": True,
            },
            {
                "name": "VelocityCommand",
                "id": 0x14,
                "direction": "tx",
                "reliable": False,
            },
        ],
    }


def test_crc8_matches_independent_reference():
    data = bytes.fromhex("11 03 00 00 07")
    expected = 0
    for byte in data:
        expected ^= byte
        for _ in range(8):
            expected = (
                ((expected << 1) ^ 0x31) & 0xFF
                if expected & 0x80
                else (expected << 1) & 0xFF
            )

    assert calculate_checksum(data, "CRC8") == expected


def test_frame_parser_handles_noise_fragments_and_bad_checksum():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    frame = build_frame(protocol, 0x12, bytes([2, 1]))
    parser = FrameParser(protocol)

    assert parser.feed(b"\x00\x11" + frame[:2]) == []
    assert parser.feed(frame[2:5]) == []
    parsed = parser.feed(frame[5:])

    assert len(parsed) == 1
    assert parsed[0].packet_id == 0x12
    assert parsed[0].payload == bytes([2, 1])

    corrupted = bytearray(frame)
    corrupted[-1] ^= 0xFF
    assert parser.feed(bytes(corrupted)) == []
    assert parser.pop_errors()


def test_state_command_alias_is_reliable_and_appends_sequence():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    command = parse_command("left", protocol)
    frame, sequence = encode_outgoing(protocol, command.message, sequence=7)

    assert command.action == "send"
    assert command.message.name == "StateMachineCommand"
    assert command.message.payload == bytes([2, 0])
    assert sequence == 7
    assert frame[2] == 0x11
    assert frame[3] == 3
    assert frame[4:-1] == bytes([2, 0, 7])


def test_arm_command_uses_little_endian_float_payload():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    command = parse_command("arm 1.25 -2 0.5 7", protocol)
    frame, sequence = encode_outgoing(protocol, command.message, sequence=3)

    assert sequence == 3
    assert frame[2] == 0x13
    assert frame[3] == 14
    assert frame[4:-2] == struct.pack("<fffB", 1.25, -2.0, 0.5, 7)
    assert frame[-2] == 3


def test_velocity_command_negates_angular_value():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    command = parse_command("vel 1.25 0.5", protocol)
    frame, sequence = encode_outgoing(protocol, command.message, sequence=0)

    assert sequence is None
    assert struct.unpack("<ff", frame[4:-1]) == (1.25, -0.5)


def test_received_callback_and_ack_are_decoded():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    parser = FrameParser(protocol)

    callback_frame = parser.feed(build_frame(protocol, 0x12, bytes([3, 1])))[0]
    ack_frame = parser.feed(build_frame(protocol, 0xFD, bytes([0x11, 7])))[0]
    callback = decode_frame(protocol, callback_frame)
    ack = decode_frame(protocol, ack_frame)

    assert callback.kind == "callback"
    assert callback.fields == {"callback_id": 3, "callback_status": 1}
    assert "executing" in callback.text
    assert ack.kind == "ack"
    assert ack.fields == {"acked_id": 0x11, "ack_seq": 7}


def test_reliable_retry_reuses_frame_until_matching_ack():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    sent = []
    now = [10.0]
    command = parse_command("grip", protocol).message
    queue = ReliableQueue(
        protocol,
        lambda entry, retry: sent.append((entry.frame, retry)),
        clock=lambda: now[0],
    )

    queue.enqueue(command)
    assert len(sent) == 1
    first_frame = sent[0][0]
    assert sent[0][1] is False

    now[0] += protocol.reliable_retry_interval_s + 1e-6
    queue.tick()
    assert sent == [(first_frame, False), (first_frame, True)]

    assert queue.handle_ack(0x11, 0) is True
    now[0] += protocol.reliable_retry_interval_s * 2
    queue.tick()
    assert len(sent) == 2


def test_load_protocol_reads_yaml_and_preserves_cli_override_inputs(tmp_path):
    mapping = protocol_mapping(
        heartbeat_interval_ms=1750,
        reliable_retry_interval_ms=225,
    )
    protocol_path = tmp_path / "protocol.yaml"
    protocol_path.write_text(yaml.safe_dump(mapping), encoding="utf-8")

    protocol = load_protocol(protocol_path)

    assert protocol.port == "/dev/test-stm32"
    assert protocol.baudrate == 230400
    assert protocol.heartbeat_interval_s == 1.75
    assert protocol.reliable_retry_interval_s == 0.225

    assert resolve_connection_options(protocol, "/dev/override", 460800) == (
        "/dev/override",
        460800,
    )


def test_heartbeat_retries_same_count_until_echo():
    protocol = ProtocolConfig.from_mapping(protocol_mapping())
    sent = []
    time_now = [0.0]
    heartbeat = HeartbeatManager(
        protocol,
        lambda count, retry: sent.append((count, retry)),
        on_timeout=lambda count: sent.append((count, "timeout")),
        clock=lambda: time_now[0],
    )

    heartbeat.start()
    time_now[0] += protocol.heartbeat_interval_s + 1e-6
    heartbeat.tick()

    assert sent == [(0, False), (0, True)]
    assert heartbeat.handle_echo(0) is True
    time_now[0] += protocol.heartbeat_interval_s / 2
    heartbeat.tick()
    assert sent == [(0, False), (0, True)]

    time_now[0] += protocol.heartbeat_interval_s / 2 + 1e-6
    heartbeat.tick()
    assert sent == [(0, False), (0, True), (1, False)]


def test_no_color_output_has_no_ansi_escape():
    assert colorize("message", "green", enabled=False) == "message"
    assert "\x1b[" in colorize("message", "green", enabled=True)
