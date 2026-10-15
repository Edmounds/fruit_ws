"""Perception selection and speech formatting, separate from mission composition."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Mapping, Optional


PRODUCE_TYPE_PREFIXES = {
    "苹果": "apple",
    "梨子": "pear",
    "南瓜": "pumpkin",
    "西红柿": "tomato",
    "辣椒": "chili",
    "洋葱": "onion",
}

PRODUCE_PREFIX_NAMES = {
    prefix: name for name, prefix in PRODUCE_TYPE_PREFIXES.items()
}

TREE_FRUIT_PREFIXES = {"apple", "pear"}
SIDE_NAMES = ("left", "right")


@dataclass(frozen=True)
class PickExecutionResult:
    """Result of the side-turn, Camera depth, and arm-target pick flow."""

    success: bool
    reason_code: str
    matched_type: Optional[str] = None


def expected_ripe_type(target_name: str) -> str:
    """Return the model class that means the target is mature."""
    try:
        prefix = PRODUCE_TYPE_PREFIXES[target_name]
    except KeyError as exc:
        raise ValueError(f"unknown target name: {target_name}") from exc
    return f"{prefix}_ripe"


def spoken_produce_name(model_type: Optional[str]) -> str:
    """Return the Chinese produce name from one model class."""
    if not model_type:
        return "未知果实"
    prefix = model_type.rsplit("_", 1)[0]
    return PRODUCE_PREFIX_NAMES.get(prefix, "未知果实")


def spoken_model_type(model_type: Optional[str]) -> str:
    """Return a Chinese phrase for a model type suitable for speech."""
    if not model_type:
        return "未知目标"
    if model_type == "bad":
        return "坏果"
    parts = model_type.rsplit("_", 1)
    if len(parts) != 2:
        return "未知目标"
    prefix, maturity = parts
    produce_name = PRODUCE_PREFIX_NAMES.get(prefix)
    if produce_name is None:
        return "未知目标"
    if maturity == "ripe":
        return f"成熟{produce_name}"
    if maturity == "unripe":
        return f"不成熟{produce_name}"
    return "未知目标"


def format_b_tree_name(position_id: int) -> str:
    """Return a spoken B-zone tree label."""
    return f"{_chinese_index(position_id)}号"


def format_position_name(position_id: int) -> str:
    """Return a spoken C-zone position name."""
    return f"{_chinese_index(position_id)}号"


def is_tree_fruit_type(model_type: str) -> bool:
    """Return whether a model class is apple or pear."""
    prefix = model_type.rsplit("_", 1)[0]
    return prefix in TREE_FRUIT_PREFIXES


def merge_seen_types(seen: Iterable[str], latest: Iterable[str]) -> List[str]:
    """Keep a stable list of types seen at least once during a window."""
    merged = list(seen)
    for item in latest:
        if item not in merged:
            merged.append(item)
    return merged


def ordered_c_tasks(c_plan: Mapping[str, object]) -> List[Mapping[str, object]]:
    """Return C tasks in QR order."""
    return list(c_plan.get("tasks", []))


def format_b_announcement(b_plan: Mapping[str, object]) -> str:
    """Build a spoken summary for the B plan."""
    parts = [
        f"{_chinese_index(int(task['position_id']))}号{task['target']}"
        for task in b_plan.get("tasks", [])
    ]
    return "；".join(parts)


def format_c_announcement(c_plan: Mapping[str, object]) -> str:
    """Build a spoken summary for the C plan."""
    parts = [
        f"{_chinese_index(int(task['position_id']))}号{task['target']}"
        for task in c_plan.get("tasks", [])
    ]
    return "；".join(parts)


def format_produce_count(produce: str, count: int) -> str:
    """Return a spoken summary phrase for one produce count, e.g. '南瓜两个'."""
    return f"{produce}{_chinese_count(count)}"


def _other_side(side: str) -> str:
    return "right" if side == "left" else "left"


def _chinese_index(value: int) -> str:
    names = {
        1: "一",
        2: "二",
        3: "三",
        4: "四",
        5: "五",
        6: "六",
        7: "七",
        8: "八",
        9: "九",
        10: "十",
        11: "十一",
        12: "十二",
    }
    return names.get(value, str(value))


def _chinese_count(value: int) -> str:
    if value == 1:
        return "一个"
    if value == 2:
        return "两个"
    if 3 <= value <= 10:
        return f"{_chinese_index(value)}个"
    if 11 <= value <= 19:
        return f"十{_chinese_index(value - 10)}个"
    if 20 <= value <= 29:
        unit = "" if value == 20 else _chinese_index(value - 20)
        return f"二十{unit}个"
    if 30 <= value <= 32:
        unit = "" if value == 30 else _chinese_index(value - 30)
        return f"三十{unit}个"
    return f"{value}个"
