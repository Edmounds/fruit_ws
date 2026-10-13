"""Pure QR payload normalization and B/C plan parsing."""

import re
from typing import Any, Dict, List, Tuple


B_VALID_TARGETS = {"苹果", "梨子"}
C_VALID_TARGETS = {"南瓜", "西红柿", "辣椒", "洋葱"}


class ParseError(ValueError):
    """Raised when a decoded QR payload does not match the expected zone format."""


def _normalized_lines(payload: str) -> List[str]:
    normalized = payload.lstrip("\ufeff").replace("\r\n", "\n").replace("\r", "\n")
    return [line.strip() for line in normalized.split("\n") if line.strip()]


def normalize_payload(payload: str) -> str:
    """Canonical text used for consecutive-frame confirmation."""
    return "\n".join(_normalized_lines(payload))


def _plan(zone: str, tasks: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "zone": zone,
        "task_count": len(tasks),
        "tasks": tasks,
    }


def _parse_b_target_names(payload: str) -> List[str]:
    lines = _normalized_lines(payload)
    if not lines:
        raise ParseError("expected at least one B QR target line")
    # If line count < 8, split lines by commas/spaces in case targets share a line
    if len(lines) < 8:
        tokens = []
        for line in lines:
            split_parts = [p.strip() for p in re.split(r"[\s,，;；]+", line) if p.strip()]
            tokens.extend(split_parts)
        if len(tokens) == 8:
            return tokens
    return lines


def build_b_plan(payload: str) -> Dict[str, Any]:
    names = _parse_b_target_names(payload)
    if len(names) != 8:
        raise ParseError(f"B QR requires exactly 8 ordered tasks, got {len(names)}")
    for idx, name in enumerate(names, start=1):
        if name not in B_VALID_TARGETS:
            raise ParseError(
                f"B QR target #{idx} '{name}' is invalid; must be 苹果 or 梨子"
            )
    tasks = [
        {
            "sequence": index,
            "position_id": index,
            "target": name,
        }
        for index, name in enumerate(names, start=1)
    ]
    return _plan("B", tasks)


def build_c_plan(payload: str) -> Dict[str, Any]:
    lines = _normalized_lines(payload)
    if not lines:
        raise ParseError("expected at least one C QR task line")

    tasks_data = _parse_c_task_lines(lines)
    if len(tasks_data) != 8:
        raise ParseError(f"C QR requires exactly 8 ordered tasks, got {len(tasks_data)}")

    positions = []
    for idx, (name, position) in enumerate(tasks_data, start=1):
        if name not in C_VALID_TARGETS:
            raise ParseError(
                f"C QR target #{idx} '{name}' is invalid; must be in {C_VALID_TARGETS}"
            )
        if not (1 <= position <= 12):
            raise ParseError(
                f"C QR task #{idx} position {position} is invalid; must be between 1 and 12"
            )
        positions.append(position)

    if len(set(positions)) != 8:
        raise ParseError(
            f"C QR positions must contain 8 unique position IDs from 1 to 12, got {positions}"
        )

    tasks = [
        {
            "sequence": index,
            "position_id": position,
            "target": name,
        }
        for index, (name, position) in enumerate(tasks_data, start=1)
    ]
    return _plan("C", tasks)


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

    target = (line[: match.start()] + line[match.end():]).strip(" \t,，;；:：-")
    if not target:
        raise ParseError(f"C QR task line has no target name: {line}")
    return target, int(match.group())
