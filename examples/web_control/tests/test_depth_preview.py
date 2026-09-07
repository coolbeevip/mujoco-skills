import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from depth_preview import colorize
from scenes import MicroduckScene


class DepthPreviewTests(unittest.TestCase):
    def test_fixed_scale_clipping_and_invalid_pixels(self):
        depths = np.array([[0.01, 0.1, 5, 8, 0, -1, np.nan, np.inf]])
        original = depths.copy()
        rgb = colorize(depths)
        self.assertEqual(rgb.dtype, np.uint8)
        self.assertEqual(rgb.shape, (1, 8, 3))
        np.testing.assert_array_equal(rgb[0, :2], [[240, 80, 40]] * 2)
        np.testing.assert_array_equal(rgb[0, 2:4], [[40, 80, 180]] * 2)
        self.assertFalse(rgb[0, 4:].any())
        np.testing.assert_array_equal(depths, original)
        np.testing.assert_array_equal(
            colorize(np.array([[1.5]]))[0, 0], colorize(np.array([[1.5, 4]]))[0, 0]
        )

    def test_depth_mode_restored_after_render_failure(self):
        scene = MicroduckScene.__new__(MicroduckScene)
        scene.runtime = Mock()
        scene.observer_renderer = Mock()
        scene.observer_renderer.render.side_effect = RuntimeError("render failed")
        with self.assertRaises(RuntimeError):
            scene.depth_preview()
        scene.observer_renderer.disable_depth_rendering.assert_called_once()
