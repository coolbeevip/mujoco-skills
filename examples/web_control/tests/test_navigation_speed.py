import unittest

from motion import Motion, forward_speed
from test_navigation import Scene


class NavigationSpeedTests(unittest.TestCase):
    def test_only_verified_clear_straights_get_faster_commands(self):
        clear = dict(floor_cm=80, front_cm=90, bearing_deg=0)
        self.assertEqual(forward_speed(30, **clear, roomy=True), 0.4)
        self.assertEqual(forward_speed(30, **clear), 0.35)
        self.assertEqual(forward_speed(30, **clear, narrow=True), 0.3)
        self.assertEqual(forward_speed(10, **clear, roomy=True), 0.3)
        for key, value in (("floor_cm", 30), ("front_cm", 35), ("bearing_deg", 12)):
            self.assertEqual(
                forward_speed(30, **{**clear, key: value}, roomy=True), 0.3
            )
        self.assertEqual(forward_speed(30), 0.3)

    def test_fast_motion_slows_before_endpoint(self):
        scene = Scene()
        motion = Motion("forward", 30, scene.motion_pose(), forward_speed=0.4)
        motion.tick(scene)
        self.assertEqual(scene.actions[-1][0], 0.4)
        scene.x = 0.23
        motion.tick(scene)
        self.assertEqual(scene.actions[-1][0], 0.3)
        self.assertEqual(motion.result()["speed_command_m_s"], 0.4)
