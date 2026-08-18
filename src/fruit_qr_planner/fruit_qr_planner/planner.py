"""Pure parsing and sequencing logic shared by the ROS node and unit tests."""

import re
from typing import Any, Dict, List, Optional, Tuple


PRODUCE_NAMES = frozenset({"苹果", "梨子", "南瓜", "西红柿", "辣椒", "洋葱"})
TREE_FRUIT_NAMES = frozenset({"苹果", "梨子"})
VEGETABLE_NAMES = PRODUCE_NAMES - TREE_FRUIT_NAMES
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


def _validate_names(names: List[str], allowed_names=PRODUCE_NAMES) -> None:
    if len(names) != 8:
        raise ParseError("expected exactly 8 produce names")
    unknown = [name for name in names if name not in allowed_names]
    if unknown:
        raise ParseError(f"unknown produce name: {unknown[0]}")


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
    _validate_names(names, TREE_FRUIT_NAMES)
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
    if len(lines) != 9:
        raise ParseError("expected 8 produce names followed by one position line")

    names = lines[:8]
    _validate_names(names, VEGETABLE_NAMES)
    position_tokens = [token.strip() for token in re.split("[,，]", lines[8])]
    if len(position_tokens) != 8 or any(not token.isdigit() for token in position_tokens):
        raise ParseError("expected exactly 8 numeric position IDs")

    positions = [int(token) for token in position_tokens]
    if len(set(positions)) != 8:
        raise ParseError("position IDs must be unique")
    if any(position < 1 or position > 12 for position in positions):
        raise ParseError("position IDs must be between 1 and 12")

    tasks = [
        {
            "sequence": index,
            "position_id": position,
            "target": name,
            "stages": list(C_STAGES),
        }
        for index, (name, position) in enumerate(zip(names, positions), start=1)
    ]
    return _plan("C", 2, tasks)


class PlanSequence:
    """Confirm decoded payloads and enforce the one-shot B-then-C workflow."""

    WAITING_FOR_B = "WAITING_FOR_B"
    WAITING_FOR_C = "WAITING_FOR_C"
    COMPLETED = "COMPLETED"

    def __init__(self, confirm_frames: int = 3) -> None:
        if confirm_frames <= 0:
            raise ValueError("confirm_frames must be greater than zero")
        self.confirm_frames = confirm_frames
        self.state = self.WAITING_FOR_B
        self._candidate: Optional[str] = None
        self._candidate_count = 0

    def observe(self, payload: str) -> Optional[Dict[str, Any]]:
        plan, _error = self.observe_with_error(payload)
        return plan

    def observe_with_error(self, payload: str) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        if self.state == self.COMPLETED:
            return None, None

        canonical_payload = "\n".join(_normalized_lines(payload))
        if canonical_payload != self._candidate:
            self._candidate = canonical_payload
            self._candidate_count = 1
        else:
            self._candidate_count += 1

        try:
            plan = self._parse_for_current_state(payload)
        except ParseError as exc:
            return None, str(exc)

        if self._candidate_count < self.confirm_frames:
            return None, None

        self._reset_candidate()
        self.state = self.WAITING_FOR_C if self.state == self.WAITING_FOR_B else self.COMPLETED
        return plan, None

    def _parse_for_current_state(self, payload: str) -> Dict[str, Any]:
        if self.state == self.WAITING_FOR_B:
            return build_b_plan(payload)
        return build_c_plan(payload)

    def _reset_candidate(self) -> None:
        self._candidate = None
        self._candidate_count = 0
