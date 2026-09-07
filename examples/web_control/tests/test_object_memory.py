"""记忆保留的是观测证据；遮挡、换任务、重启不能伪造当前可见性。"""

import tempfile
import unittest
from pathlib import Path

from object_memory import ObjectMemory


def ball(color="purple", position=None):
    return dict(
        object_id=f"ball:{color}",
        name=color + " ball",
        color=color,
        kind="ball",
        source="head_rgbd",
        position=position,
    )


class ObjectMemoryTests(unittest.TestCase):
    def test_occlusion_keeps_position_and_updates_other_objects(self):
        memory = ObjectMemory()
        self.addCleanup(memory.close)
        memory.begin_scene("office")
        memory.observe([ball(position=[2, -2]), ball("red", [0, 1])], 1, [0, 0, 0])
        memory.observe([ball("red", [0.1, 1])], 120, [1, 0, 0])
        purple = memory.target("purple")
        self.assertEqual(purple["position"], [2, -2])
        self.assertFalse(purple["currently_visible"])
        self.assertEqual(len(memory.current()), 2)
        self.assertEqual(memory.target("red")["position"], [0.1, 1])

    def test_missing_depth_does_not_overwrite_last_verified_position(self):
        memory = ObjectMemory()
        self.addCleanup(memory.close)
        memory.begin_scene("office")
        memory.observe([ball(position=[2, -2])], 1, [0, 0, 0])
        memory.observe([ball()], 10, [1, 0, 0])
        record = memory.target("purple")
        self.assertEqual(record["position"], [2, -2])
        self.assertEqual(record["position_observed_at_s"], 1)
        self.assertFalse(record["position_verified_now"])

    def test_restart_preserves_archive_but_not_current_world_positions(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "objects.sqlite3"
            memory = ObjectMemory(path)
            memory.begin_scene("office")
            memory.observe([ball(position=[2, -2])], 1, [0, 0, 0])
            memory.observe([ball(position=[2.1, -2])], 2, [0, 0, 0])
            memory.close()
            memory = ObjectMemory(path)
            self.addCleanup(memory.close)
            memory.begin_scene("office")
            self.assertIsNone(memory.target("purple"))
            record = memory.archive()[0]
            self.assertTrue(record["historical"])
            self.assertFalse(record["currently_visible"])
            self.assertEqual(record["position"], [2.1, -2])
            self.assertEqual(
                memory.db.execute("SELECT COUNT(*) FROM observations").fetchone()[0], 2
            )
