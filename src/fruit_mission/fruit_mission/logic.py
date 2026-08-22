"""Pure mission decision helpers for A/B/C perception tasks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional


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
class SideObservation:
    """Latest model labels seen by left and right side cameras."""

    left: List[str]
    right: List[str]


@dataclass(frozen=True)
class PickDecision:
    """One side-pick decision from an observation."""

    should_pick: bool
    side: Optional[str]
    reason: str
    matched_type: Optional[str] = None
    reason_code: str = ""


@dataclass(frozen=True)
class PickExecutionResult:
    """Result of an A-zone open-loop or C-zone closed-loop pick."""

    success: bool
    reason_code: str
    matched_type: Optional[str] = None
    safe_to_continue: bool = True


@dataclass(frozen=True)
class BTreeDecision:
    """B-zone species comparison and single-target selection result."""

    reason_code: str
    observed_name: Optional[str] = None
    target_index: Optional[int] = None


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
        return f"未成熟{produce_name}"
    return "未知目标"


def decide_b_tree_targets(
    target_name: str,
    expected_type: str,
    candidates: Iterable[tuple[str, float, float]],
) -> BTreeDecision:
    """Compare one B-tree result and select at most one ripe target."""
    candidate_list = list(candidates)
    types = [model_type for model_type, _x, _y in candidate_list]
    if not types:
        return BTreeDecision("no_detection")

    observed_names = {spoken_produce_name(model_type) for model_type in types}
    if len(observed_names) != 1 or "未知果实" in observed_names:
        return BTreeDecision("inconsistent_types")

    observed_name = observed_names.pop()
    if observed_name != target_name:
        return BTreeDecision("type_mismatch", observed_name)

    ripe_indices = tuple(
        index for index, model_type in enumerate(types)
        if model_type == expected_type
    )
    if not ripe_indices:
        return BTreeDecision("no_ripe_target", observed_name)
    target_index = max(
        ripe_indices,
        key=lambda index: abs(candidate_list[index][1]) - candidate_list[index][2],
    )
    return BTreeDecision("matched", observed_name, target_index)


def format_position_name(position_id: int) -> str:
    """Return a spoken C-zone position name."""
    return f"{_chinese_index(position_id)}号位置"


def choose_maturity_side(
    observation: SideObservation,
    preferred_side: str = "left",
) -> PickDecision:
    """Choose a side from A-zone ripe/unripe labels."""
    if preferred_side not in SIDE_NAMES:
        raise ValueError("preferred_side must be left or right")

    candidates = {"left": observation.left, "right": observation.right}
    for side in (preferred_side, _other_side(preferred_side)):
        if "ripe" in candidates[side]:
            return PickDecision(True, side, "matched ripe target", "ripe", "matched")

    if not observation.left and not observation.right:
        return PickDecision(
            False,
            None,
            "no target detected",
            reason_code="no_detection",
        )
    return PickDecision(
        False,
        None,
        "only unripe targets",
        reason_code="target_unripe",
    )


def choose_side(
    observation: SideObservation,
    expected_type: Optional[str] = None,
    preferred_side: str = "left",
) -> PickDecision:
    """Choose at most one side to pick from the latest side-camera output."""
    if preferred_side not in SIDE_NAMES:
        raise ValueError("preferred_side must be left or right")

    candidates: Dict[str, List[str]] = {
        "left": list(observation.left),
        "right": list(observation.right),
    }
    for side in (preferred_side, _other_side(preferred_side)):
        types = candidates[side]
        matched = _first_pickable_type(types, expected_type)
        if matched:
            return PickDecision(True, side, "matched ripe target", matched, "matched")

    reason_code = _skip_reason(candidates.values(), expected_type)
    if expected_type:
        return PickDecision(
            False,
            None,
            f"no {expected_type} detected",
            reason_code=reason_code,
        )
    return PickDecision(
        False,
        None,
        "no ripe target detected",
        reason_code=reason_code,
    )


def without_tree_fruits(observation: SideObservation) -> SideObservation:
    """Return A/C candidates with apple and pear removed."""
    return SideObservation(
        left=[item for item in observation.left if not is_tree_fruit_type(item)],
        right=[item for item in observation.right if not is_tree_fruit_type(item)],
    )


def only_tree_fruits(observation: SideObservation) -> SideObservation:
    """Return B-zone candidates, keeping only apple and pear."""
    return SideObservation(
        left=[item for item in observation.left if is_tree_fruit_type(item)],
        right=[item for item in observation.right if is_tree_fruit_type(item)],
    )


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
    """Build a compact spoken summary for the B plan."""
    parts = [
        f"{int(task['position_id'])}{task['target']}"
        for task in b_plan.get("tasks", [])
    ]
    return "，".join(parts)


def format_c_announcement(c_plan: Mapping[str, object]) -> str:
    """Build a compact spoken summary for the C plan."""
    parts = [
        (
            f"{int(task['sequence'])}，"
            f"{int(task['position_id'])}{task['target']}"
        )
        for task in c_plan.get("tasks", [])
    ]
    return "，".join(parts)


def _first_pickable_type(
    types: Iterable[str],
    expected_type: Optional[str],
) -> Optional[str]:
    for item in types:
        if expected_type:
            if item == expected_type:
                return item
            continue
        if item.endswith("_ripe"):
            return item
    return None


def _skip_reason(
    side_types: Iterable[Iterable[str]],
    expected_type: Optional[str],
) -> str:
    types = [item for group in side_types for item in group]
    if not types:
        return "no_detection"

    if expected_type:
        prefix = expected_type.rsplit("_", 1)[0]
        if f"{prefix}_unripe" in types:
            return "target_unripe"
        return "type_mismatch"

    if any(item.endswith("_unripe") for item in types):
        return "only_unripe_targets"
    return "no_ripe_target"


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
