"""任务层纯逻辑测试。不启动 ROS、不动臂。

跑法：
    cd src/roarm_task && python3 -m unittest discover -s test -v
"""

import os
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from roarm_task import config, link, primitives, sequence  # noqa: E402
from roarm_task.scene import Scene  # noqa: E402


def ok(action="stub"):
    return link.Result(action=action, rc=0, detail="stub", elapsed=0.01)


def fail(action="stub", rc=1):
    return link.Result(action=action, rc=rc, detail="stub-fail", elapsed=0.01)


class TestPrimitiveGeometry(unittest.TestCase):
    def test_approach_adds_standoff(self):
        with patch.object(link, "move", return_value=ok()) as m:
            primitives.approach((1.0, 2.0, 3.0), standoff=0.05)
            self.assertEqual(m.call_args[0][:3], (1.0, 2.0, 3.05))

    def test_descend_keeps_target(self):
        with patch.object(link, "move", return_value=ok()) as m:
            primitives.descend((0.1, 0.2, 0.3))
            self.assertEqual(m.call_args[0][:3], (0.1, 0.2, 0.3))

    def test_lift_default_height(self):
        with patch.object(link, "move", return_value=ok()) as m:
            primitives.lift((0.1, 0.2, 0.3))
            self.assertAlmostEqual(m.call_args[0][2], 0.3 + config.LIFT_M)


class TestGripperSemantics(unittest.TestCase):
    def test_grasp_maps_to_close_fingers(self):
        with patch.object(link, "grip", return_value=ok()) as m:
            primitives.grasp()
            m.assert_called_once_with("close_fingers", action="grasp")

    def test_release_maps_to_open_fingers(self):
        with patch.object(link, "grip", return_value=ok()) as m:
            primitives.release()
            m.assert_called_once_with("open_fingers", action="release")

    def test_measured_mapping(self):
        # 2026-10-18 实物实测：CLI "open" -> 舵机 100° -> 两指张开
        #                     CLI "close"-> 舵机 21.9° -> 两指合拢
        # （10-17 从仿真推出来的"名字相反"是错的，仿真的夹爪桥做过 clamp，方向没验过）
        self.assertEqual(config.GRIP_CMD["close_fingers"], "close")
        self.assertEqual(config.GRIP_CMD["open_fingers"], "open")

    def test_gap_conversion(self):
        # 2026-10-18 卡尺实测：0.000 -> 65 mm，-0.025 -> 8 mm（合到底仍有 8 mm 间隙）
        self.assertAlmostEqual(primitives.gripper_gap_mm(0.0), 65.0)
        self.assertAlmostEqual(primitives.gripper_gap_mm(-0.025), 8.0)
        self.assertAlmostEqual(primitives.gripper_gap_mm(-0.015), 30.8, places=1)
        self.assertIsNone(primitives.gripper_gap_mm(None))


class TestGraspCheck(unittest.TestCase):
    def test_empty_grasp_detected(self):
        # -0.025 = 合拢到底（间距 0 mm）
        with patch.object(link, "gripper_state", return_value=-0.025):
            grabbed, _pos, note = primitives.check_grasp(0.03)
        self.assertFalse(grabbed)
        self.assertIn("夹空", note)

    def test_grasped_detected(self):
        # 物宽 30 mm -> 间距约 30 mm -> joint7_left 约 -0.0154
        with patch.object(link, "gripper_state", return_value=-0.0154):
            grabbed, _pos, note = primitives.check_grasp(0.03)
        self.assertTrue(grabbed)
        self.assertIn("夹住", note)

    def test_too_wide_is_not_grasped(self):
        # 0.0 = 全开 50 mm，明显大于物宽 30 mm
        with patch.object(link, "gripper_state", return_value=0.0):
            grabbed, _pos, note = primitives.check_grasp(0.03)
        self.assertFalse(grabbed)
        self.assertIn("没夹到", note)

    def test_unreadable_returns_none(self):
        with patch.object(link, "gripper_state", return_value=None):
            grabbed, _pos, _note = primitives.check_grasp(0.03)
        self.assertIsNone(grabbed)


class TestScene(unittest.TestCase):
    def test_default_scene_valid(self):
        self.assertEqual(Scene.default().problems(), [])

    def test_lift_height(self):
        sc = Scene(pick=(0.3, 0.0, 0.1), place=(0.4, 0.0, 0.2),
                   object_height=0.05, clearance=0.06)
        self.assertAlmostEqual(sc.lift_height, 0.11)

    def test_zero_height_rejected(self):
        sc = Scene(pick=(0.3, 0.0, 0.1), place=(0.4, 0.0, 0.2), object_height=0.0)
        self.assertTrue(any("object_height" in p for p in sc.problems()))

    def test_coincident_points_rejected(self):
        sc = Scene(pick=(0.3, 0.0, 0.1), place=(0.3, 0.0, 0.1))
        self.assertTrue(any("几乎重合" in p for p in sc.problems()))

    def test_overrides_ignore_none(self):
        sc = Scene.default().with_overrides(name="x", object_height=None)
        self.assertEqual(sc.name, "x")
        self.assertEqual(sc.object_height, Scene.default().object_height)


class TestGraspWithRetry(unittest.TestCase):
    def _scene(self, verify=True, width=0.03):
        return Scene(pick=(0.3, -0.3, 0.074), place=(0.4, 0.1, 0.2),
                     object_width=width, verify_grasp=verify)

    def test_no_verify_passes_through(self):
        with patch.object(primitives, "set_grip_frac", return_value=ok()) as g, \
             patch.object(primitives, "check_grasp") as chk:
            res = sequence.grasp_with_retry(self._scene(verify=False))()
        self.assertTrue(res.ok)
        g.assert_called_once()
        chk.assert_not_called()

    def test_verified_grasp_ok(self):
        with patch.object(primitives, "set_grip_frac", return_value=ok()), \
             patch.object(primitives, "check_grasp",
                          return_value=(True, -0.015, "夹住")):
            res = sequence.grasp_with_retry(self._scene())()
        self.assertTrue(res.ok)
        self.assertIn("夹住", res.detail)

    def test_empty_then_retry_succeeds(self):
        seq = [(False, 0.0, "夹空"), (True, -0.015, "夹住")]
        with patch.object(primitives, "set_grip_frac", return_value=ok()), \
             patch.object(primitives, "set_grip_frac", return_value=ok()), \
             patch.object(primitives, "approach", return_value=ok()), \
             patch.object(primitives, "descend", return_value=ok()), \
             patch.object(primitives, "check_grasp", side_effect=seq):
            res = sequence.grasp_with_retry(self._scene())()
        self.assertTrue(res.ok)
        self.assertIn("重试夹住", res.detail)

    def test_empty_twice_gives_up(self):
        with patch.object(primitives, "set_grip_frac", return_value=ok()), \
             patch.object(primitives, "set_grip_frac", return_value=ok()), \
             patch.object(primitives, "approach", return_value=ok()), \
             patch.object(primitives, "descend", return_value=ok()), \
             patch.object(primitives, "check_grasp",
                          return_value=(False, 0.0, "夹空")):
            res = sequence.grasp_with_retry(self._scene())()
        self.assertFalse(res.ok)
        self.assertIn("未夹住", res.detail)


class TestSequence(unittest.TestCase):
    def test_default_sequence_shape(self):
        steps = sequence.build_pick_place()
        self.assertEqual(len(steps), 11)   # 「张开夹爪」是新增的第 2 步
        self.assertEqual([s.phase for s in steps][:4],
                         [sequence.Phase.PREPARE, sequence.Phase.PREPARE,
                          sequence.Phase.APPROACH, sequence.Phase.DESCEND])
        self.assertEqual(steps[4].name, "合爪夹持")
        self.assertIs(steps[4].phase, sequence.Phase.GRASP)

    def test_coordinates_follow_scene(self):
        sc = Scene(pick=(0.31, -0.30, 0.08), place=(0.45, 0.05, 0.21),
                   standoff=0.07, object_height=0.04, clearance=0.05)
        steps = sequence.build_pick_place(sc)
        with patch.object(link, "move", return_value=ok()) as m:
            steps[2].fn()   # 接近 = pick.z + standoff = 0.15
            for got, want in zip(m.call_args[0][:3], (0.31, -0.30, 0.15)):
                self.assertAlmostEqual(got, want)
            steps[5].fn()   # 抬起 = pick.z + 4 + 5 cm = 0.17
            self.assertAlmostEqual(m.call_args[0][2], 0.17)

    def test_stops_at_failure(self):
        calls = []
        steps = [
            sequence.Step("a", sequence.Phase.PREPARE,
                          lambda: (calls.append("a"), ok())[1]),
            sequence.Step("b", sequence.Phase.DESCEND,
                          lambda: (calls.append("b"), fail())[1]),
            sequence.Step("c", sequence.Phase.LIFT,
                          lambda: (calls.append("c"), ok())[1]),
        ]
        with patch.object(primitives, "go_home", return_value=ok()):
            result = sequence.run(steps)
        self.assertFalse(result.ok)
        self.assertEqual(result.failed_at, "b")
        self.assertIs(result.failed_phase, sequence.Phase.DESCEND)
        self.assertEqual(calls, ["a", "b"])

    def test_home_after_failure(self):
        steps = [sequence.Step("b", sequence.Phase.GRASP, lambda: fail())]
        with patch.object(primitives, "go_home", return_value=ok()) as m:
            result = sequence.run(steps)
        m.assert_called_once()
        self.assertTrue(result.recovered)


if __name__ == "__main__":
    unittest.main(verbosity=2)
