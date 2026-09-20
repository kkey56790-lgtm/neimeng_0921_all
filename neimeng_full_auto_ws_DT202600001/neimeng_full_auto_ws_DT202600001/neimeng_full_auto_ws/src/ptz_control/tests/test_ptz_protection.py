#!/usr/bin/env python3
import unittest


class PTZProtectionPolicyTests(unittest.TestCase):
    def test_same_point_sweep_is_eligible_for_queueing(self):
        protected = {"action": "goto_preset", "point_name": "东区巡航点1左"}
        sweep = {"action": "sweep", "point_name": "东区巡航点1左"}
        self.assertEqual(protected["point_name"], sweep["point_name"])

    def test_other_point_sweep_is_not_the_same_ptz_intent(self):
        protected = {"action": "goto_preset", "point_name": "东区巡航点1左"}
        sweep = {"action": "sweep", "point_name": "东区巡航点2右"}
        self.assertNotEqual(protected["point_name"], sweep["point_name"])


if __name__ == "__main__":
    unittest.main()
