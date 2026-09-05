import unittest

try:
    import numpy as np
    from proximity import measure, target_color
except ImportError:
    np = None


@unittest.skipIf(np is None, "RGB-D 数值检查需要 MicroDuck 虚拟环境中的 NumPy")
class MeasureTests(unittest.TestCase):
    def sample(self, distance):
        rgb = np.zeros((20, 20, 3), dtype=np.uint8)
        rgb[6:14, 6:14, 2] = 200
        depth = np.full((20, 20), distance)
        rotation = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        return rgb, depth, rotation

    def test_metric_depth_and_arrival(self):
        for distance, near in ((0.25, True), (0.8, False)):
            rgb, depth, rotation = self.sample(distance)
            result = measure(rgb, depth, 60, np.zeros(3), rotation, (0, 0, 0), "blue")
            self.assertEqual(result["surface_distance_cm"], distance * 100)
            self.assertEqual(result["near"], near)

    def test_invalid_depth_does_not_claim_visible(self):
        rgb, depth, rotation = self.sample(float("nan"))
        self.assertFalse(
            measure(rgb, depth, 60, np.zeros(3), rotation, (0, 0, 0), "blue")["visible"]
        )

    def test_color_and_task_scope(self):
        self.assertEqual(target_color("走到蓝色圆柱面前"), "blue")
        self.assertIsNone(target_color("红色和蓝色之间"))
        self.assertIsNone(target_color("坐下"))


if __name__ == "__main__":
    unittest.main()
