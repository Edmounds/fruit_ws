import json
import unittest

from fruit_qr_planner.planner import (
    ParseError,
    PlanSequence,
    build_b_plan,
    build_c_plan,
)


class PlannerTest(unittest.TestCase):
    def test_builds_b_plan_in_tree_order(self):
        payload = "苹果\n梨子\n南瓜\n西红柿\n辣椒\n洋葱\n苹果\n梨子"

        plan = build_b_plan(payload)

        self.assertEqual(plan["zone"], "B")
        self.assertEqual(plan["scan_index"], 1)
        self.assertEqual(plan["task_count"], 8)
        self.assertEqual(
            [(task["sequence"], task["position_id"], task["target"]) for task in plan["tasks"]],
            [
                (1, 1, "苹果"),
                (2, 2, "梨子"),
                (3, 3, "南瓜"),
                (4, 4, "西红柿"),
                (5, 5, "辣椒"),
                (6, 6, "洋葱"),
                (7, 7, "苹果"),
                (8, 8, "梨子"),
            ],
        )
        self.assertEqual(
            plan["tasks"][0]["stages"],
            ["navigate", "identify", "announce", "pick_or_skip", "advance"],
        )

    def test_rejects_invalid_b_payload(self):
        with self.assertRaises(ParseError):
            build_b_plan("苹果\n梨子")
        with self.assertRaises(ParseError):
            build_b_plan("苹果\n梨子\n南瓜\n西红柿\n辣椒\n洋葱\n苹果\n香蕉")

    def test_builds_c_plan_by_pairing_names_and_positions(self):
        payload = (
            "洋葱\n南瓜\n西红柿\n辣椒\n苹果\n梨子\n洋葱\n南瓜\n"
            "4, 3, 1, 10, 8, 9, 2, 11"
        )

        plan = build_c_plan(payload)

        self.assertEqual(plan["zone"], "C")
        self.assertEqual(plan["scan_index"], 2)
        self.assertEqual(plan["task_count"], 8)
        self.assertEqual(
            [(task["target"], task["position_id"]) for task in plan["tasks"]],
            [
                ("洋葱", 4),
                ("南瓜", 3),
                ("西红柿", 1),
                ("辣椒", 10),
                ("苹果", 8),
                ("梨子", 9),
                ("洋葱", 2),
                ("南瓜", 11),
            ],
        )
        self.assertEqual(
            plan["tasks"][0]["stages"],
            ["navigate", "stable_stop", "identify", "announce", "pick_or_skip", "advance"],
        )

    def test_rejects_invalid_c_payload_as_a_whole(self):
        base = "洋葱\n南瓜\n西红柿\n辣椒\n苹果\n梨子\n洋葱\n南瓜\n"
        for positions in ("4,3,1", "4,3,1,10,8,9,2,2", "4,3,1,10,8,9,2,13"):
            with self.subTest(positions=positions):
                with self.assertRaises(ParseError):
                    build_c_plan(base + positions)

    def test_normalizes_bom_crlf_and_ascii_commas(self):
        payload = (
            "\ufeff洋葱\r\n南瓜\r\n西红柿\r\n辣椒\r\n苹果\r\n梨子\r\n洋葱\r\n南瓜\r\n"
            "4，3，1，10，8，9，2，11"
        )

        self.assertEqual(build_c_plan(payload)["tasks"][3]["position_id"], 10)

    def test_plan_json_is_utf8_readable(self):
        plan = build_b_plan("苹果\n梨子\n南瓜\n西红柿\n辣椒\n洋葱\n苹果\n梨子")

        encoded = json.dumps(plan, ensure_ascii=False, separators=(",", ":"))

        self.assertIn('"target":"苹果"', encoded)


class PlanSequenceTest(unittest.TestCase):
    def test_confirms_b_then_c_after_three_matching_frames(self):
        sequence = PlanSequence(confirm_frames=3)
        b_payload = "苹果\n梨子\n南瓜\n西红柿\n辣椒\n洋葱\n苹果\n梨子"
        c_payload = (
            "洋葱\n南瓜\n西红柿\n辣椒\n苹果\n梨子\n洋葱\n南瓜\n"
            "4,3,1,10,8,9,2,11"
        )

        self.assertIsNone(sequence.observe(b_payload))
        self.assertIsNone(sequence.observe(b_payload))
        b_plan = sequence.observe(b_payload)
        self.assertEqual(b_plan["zone"], "B")
        self.assertEqual(sequence.state, PlanSequence.WAITING_FOR_C)

        self.assertIsNone(sequence.observe(c_payload))
        self.assertIsNone(sequence.observe(c_payload))
        c_plan = sequence.observe(c_payload)
        self.assertEqual(c_plan["zone"], "C")
        self.assertEqual(sequence.state, PlanSequence.COMPLETED)
        self.assertIsNone(sequence.observe(c_payload))

    def test_invalid_or_changed_frame_resets_confirmation(self):
        sequence = PlanSequence(confirm_frames=3)
        first = "苹果\n梨子\n南瓜\n西红柿\n辣椒\n洋葱\n苹果\n梨子"
        second = "梨子\n苹果\n南瓜\n西红柿\n辣椒\n洋葱\n苹果\n梨子"

        self.assertIsNone(sequence.observe(first))
        self.assertIsNone(sequence.observe(first))
        self.assertIsNone(sequence.observe(""))
        self.assertIsNone(sequence.observe(first))
        self.assertIsNone(sequence.observe(second))
        self.assertIsNone(sequence.observe(second))
        self.assertEqual(sequence.observe(second)["tasks"][0]["target"], "梨子")

    def test_c_payload_cannot_publish_before_b(self):
        sequence = PlanSequence(confirm_frames=1)
        c_payload = (
            "洋葱\n南瓜\n西红柿\n辣椒\n苹果\n梨子\n洋葱\n南瓜\n"
            "4,3,1,10,8,9,2,11"
        )

        self.assertIsNone(sequence.observe(c_payload))
        self.assertEqual(sequence.state, PlanSequence.WAITING_FOR_B)

    def test_requires_positive_confirmation_count(self):
        with self.assertRaises(ValueError):
            PlanSequence(confirm_frames=0)

    def test_exposes_parse_error_without_advancing_state(self):
        sequence = PlanSequence(confirm_frames=3)

        plan, error = sequence.observe_with_error("苹果\n梨子")

        self.assertIsNone(plan)
        self.assertIn("8 produce names", error)
        self.assertEqual(sequence.state, PlanSequence.WAITING_FOR_B)


if __name__ == "__main__":
    unittest.main()
