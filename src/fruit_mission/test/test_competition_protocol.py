import unittest

from fruit_mission.competition_protocol import (
    count_result, localization_sample_ready, produce_type,
)


class CompetitionProtocolTest(unittest.TestCase):
    def test_type_groups_maturity_under_chinese_name(self):
        self.assertEqual(produce_type('apple_ripe'), '苹果')
        self.assertEqual(produce_type('pumpkin_unripe'), '南瓜')
        self.assertEqual(produce_type(None), 'unknown')

    def test_counts_only_actual_result(self):
        statistics = {}
        count_result(statistics, 'A', 'chili_ripe', 'completed')
        count_result(statistics, 'A', None, 'skipped')
        self.assertEqual(statistics['A']['辣椒']['completed'], 1)
        self.assertEqual(statistics['A']['unknown']['skipped'], 1)

    def test_localization_sample_requires_new_finite_pose(self):
        values = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.assertTrue(localization_sample_ready(
            2_000_000_000, 2_500_000_000, 1_000_000_000, values))
        self.assertFalse(localization_sample_ready(
            900_000_000, 2_500_000_000, 1_000_000_000, values))


if __name__ == '__main__':
    unittest.main()
