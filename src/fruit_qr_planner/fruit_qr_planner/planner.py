"""Pure QR payload normalization and B/C plan parsing."""

import re
from typing import Any, Dict, List, Tuple


B_STAGES = ["navigate", "identify", "announce", "pick_or_skip", "advance"]
C_STAGES = [
    "navigate",
    "stable_stop",
    "identify",
    "announce",
    "pick_or_skip",
    "advance",
]


class ParseError(ValueError):
    """Raised when a decoded QR payload does not match the expected zone format."""


def _normalized_lines(payload: str) -> List[str]:
    normalized = payload.lstrip("\ufeff").replace("\r\n", "\n").replace("\r", "\n")
    return [line.strip() for line in normalized.split("\n") if line.strip()]


def normalize_payload(payload: str) -> str:
    """Canonical text used for consecutive-frame confirmation."""
    return "\n".join(_normalized_lines(payload))


def _plan(zone: str, scan_index: int, tasks: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "zone": zone,
        "scan_index": scan_index,
        "task_count": len(tasks),
        "tasks": tasks,
    }


def build_b_plan(payload: str) -> Dict[str, Any]:
    names = _normalized_lines(payload)
    if not names:
        raise ParseError("expected at least one B QR target line")
    tasks = [
        {
            "sequence": index,
            "position_id": index,
            "target": name,
            "stages": list(B_STAGES),
        }
        for index, name in enumerate(names, start=1)
    ]
    return _plan("B", 1, tasks)


def build_c_plan(payload: str) -> Dict[str, Any]:
    lines = _normalized_lines(payload)
    if not lines:
        raise ParseError("expected at least one C QR task line")

    tasks_data = _parse_c_task_lines(lines)
    tasks = [
        {
            "sequence": index,
            "position_id": position,
            "target": name,
            "stages": list(C_STAGES),
        }
        for index, (name, position) in enumerate(tasks_data, start=1)
    ]
    return _plan("C", 2, tasks)


def _parse_c_task_lines(lines: List[str]) -> List[Tuple[str, int]]:
    position_tokens = _split_position_tokens(lines[-1])
    if len(lines) > 1 and len(position_tokens) == len(lines) - 1:
        return list(zip(lines[:-1], position_tokens))
    return [_parse_c_task_line(line) for line in lines]


def _split_position_tokens(line: str) -> List[int]:
    tokens = [token for token in re.split(r"[\s,，;；]+", line.strip()) if token]
    if not tokens or any(not token.isdigit() for token in tokens):
        return []
    return [int(token) for token in tokens]


def _parse_c_task_line(line: str) -> Tuple[str, int]:
    match = re.search(r"\d+", line)
    if match is None:
        raise ParseError(f"C QR task line has no position ID: {line}")

    target = (line[: match.start()] + line[match.end() :]).strip(" \t,，;；:：-")
    if not target:
        raise ParseError(f"C QR task line has no target name: {line}")
    return target, int(match.group())
