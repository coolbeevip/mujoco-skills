import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motion import Motion


class Plant:
    def __init__(self, yaw=0, recoil=0, stuck=False):
        self.x, self.y, self.yaw = 0, 0, yaw
        self.recoil, self.stuck = recoil, stuck
        self.last_turn = 0
        self.stops = 0

    def motion_pose(self):
        return self.x, self.y, self.yaw

    def stop(self):
        self.stops += 1

    def motion_step(self, command):
        if self.stuck:
            return
        self.x += command[0] * 0.02 * math.cos(self.yaw)
        self.y += command[0] * 0.02 * math.sin(self.yaw)
        if not any(command) and self.last_turn:
            self.yaw -= math.copysign(math.radians(self.recoil), self.last_turn)
        self.last_turn = command[2]
        self.yaw = math.remainder(self.yaw + command[2] * 0.02, math.tau)


class MotionTests(unittest.TestCase):
    def run_goal(self, plant, action, amount):
        motion = Motion(action, amount, plant.motion_pose())
        for _ in range(650):
            motion.tick(plant)
            if motion.status != "running":
                break
        return motion

    def test_forward_measured_in_start_heading_not_world_x(self):
        motion = self.run_goal(Plant(yaw=math.pi / 2), "forward", 10)
        self.assertEqual(motion.status, "reached")
        self.assertAlmostEqual(motion.distance, 10, delta=2.5)

    def test_turn_crosses_wrap_boundary(self):
        motion = self.run_goal(Plant(yaw=math.pi - 0.05), "left", 30)
        self.assertEqual(motion.status, "reached")
        self.assertAlmostEqual(motion.angle, 30, delta=5)

    def test_rechecks_after_recoil_and_corrects(self):
        motion = self.run_goal(Plant(recoil=10), "right", 30)
        self.assertEqual(motion.status, "reached")
        self.assertAlmostEqual(motion.angle, -30, delta=5)
        # 回摆后必须再次停步复核；不再用固定总等待时长推断是否发生补偿。
        self.assertGreaterEqual(motion.total_settle_steps, 40)

    def test_stuck_is_not_reported_as_completed(self):
        plant = Plant(stuck=True)
        motion = self.run_goal(plant, "forward", 10)
        self.assertEqual(motion.status, "incomplete")
        self.assertEqual(motion.steps, 500)
        self.assertGreater(plant.stops, 0)

    def test_turn_displacement_limit(self):
        plant = Plant()
        motion = Motion("left", 30, plant.motion_pose())
        plant.x = 0.25
        motion.tick(plant)
        self.assertEqual(motion.status, "incomplete")

    def test_invalid_feedback_and_targets(self):
        for action, amount, pose in [
            ("left", 90, (0, 0, 0)),
            ("forward", True, (0, 0, 0)),
            ("forward", 10, (0, 0, float("nan"))),
        ]:
            with self.assertRaises(ValueError):
                Motion(action, amount, pose)

    def test_stable_plant_can_finish_settling_early(self):
        motion = self.run_goal(Plant(), "forward", 30)
        self.assertEqual(motion.status, "reached")
        self.assertEqual(motion.total_settle_steps, 20)

    def test_unstable_body_cannot_finish_settling_early(self):
        plant = Plant()
        plant.body_stable = lambda: False
        motion = self.run_goal(plant, "forward", 30)
        self.assertEqual(motion.total_settle_steps, 50)

    def test_continuous_extension_has_one_stop_and_bounded_length(self):
        plant = Plant()
        motion = Motion("forward", 30, plant.motion_pose())
        while motion.distance < 25:
            motion.tick(plant)
        stops = plant.stops
        motion.extend(30)
        self.assertEqual(plant.stops, stops)
        with self.assertRaises(ValueError):
            motion.extend(5)
        while motion.status == "running":
            motion.tick(plant)
        self.assertEqual(motion.status, "reached")
        self.assertAlmostEqual(motion.distance, 60, delta=2.5)
        self.assertEqual(motion.total_settle_steps, 20)

    def test_speed_input_is_bounded(self):
        for speed in (0.5, 0, float("nan"), True):
            with self.assertRaises(ValueError):
                Motion("forward", 30, (0, 0, 0), forward_speed=speed)

    def test_final_check_uses_real_target_not_old_recoil_aim(self):
        plant = Plant()
        motion = Motion("left", 13, plant.motion_pose())
        plant.yaw = math.radians(14)
        motion.steps, motion.aim = 450, 35
        while motion.status == "running":
            motion.tick(plant)
        self.assertEqual(motion.status, "reached")
        self.assertLess(motion.steps, 500)
        self.assertGreaterEqual(motion.total_settle_steps, 20)


if __name__ == "__main__":
    unittest.main()
