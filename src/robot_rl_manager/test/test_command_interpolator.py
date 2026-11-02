"""CommandInterpolator 单元测试（纯逻辑，不依赖 ROS 运行时）。

覆盖 A1 命令发布插值的关键性质：线性逼近、跨段连续、单帧阶跃被摊平、
饱和保持、维度不匹配回退、未设目标返回 None、零阶基线同步。
"""

import unittest

from robot_rl_manager.rl_manager_node import CommandInterpolator


class TestCommandInterpolator(unittest.TestCase):
    def test_next_positions_none_before_target(self):
        ci = CommandInterpolator(4)
        self.assertIsNone(ci.next_positions())

    def test_linear_ramp_reaches_target(self):
        ci = CommandInterpolator(4)
        ci.mark_published([0.0])
        ci.set_target([1.0])
        # 4 帧内应从 0 线性逼近到 1，第 4 帧到达目标。
        outputs = [ci.next_positions()[0] for _ in range(4)]
        self.assertAlmostEqual(outputs[0], 0.25, places=6)
        self.assertAlmostEqual(outputs[1], 0.50, places=6)
        self.assertAlmostEqual(outputs[2], 0.75, places=6)
        self.assertAlmostEqual(outputs[3], 1.00, places=6)

    def test_saturation_holds_target(self):
        ci = CommandInterpolator(2)
        ci.mark_published([0.0])
        ci.set_target([1.0])
        ci.next_positions()
        ci.next_positions()
        # 超过 total_frames 后 alpha 饱和到 1，保持目标不越界。
        self.assertAlmostEqual(ci.next_positions()[0], 1.0, places=6)
        self.assertAlmostEqual(ci.next_positions()[0], 1.0, places=6)

    def test_single_step_is_smoothed(self):
        # 复现日志机制：策略相邻两帧踝 roll 跳 -0.36 rad，
        # 插值应把它摊成 4 帧、每帧 |Δ| <= 0.36/4，杜绝单帧大阶跃。
        ci = CommandInterpolator(4)
        ci.mark_published([0.433])
        ci.set_target([0.433 - 0.36])
        prev = 0.433
        max_step = 0.0
        for _ in range(4):
            cur = ci.next_positions()[0]
            max_step = max(max_step, abs(cur - prev))
            prev = cur
        self.assertLessEqual(max_step, 0.36 / 4 + 1e-9)
        self.assertAlmostEqual(prev, 0.433 - 0.36, places=6)

    def test_continuity_across_segments(self):
        # 一段结束时的发布位置应成为下一段起点，跨段无跳变。
        ci = CommandInterpolator(2)
        ci.mark_published([0.0])
        ci.set_target([1.0])
        ci.next_positions()
        last = ci.next_positions()[0]  # 到达 1.0
        ci.set_target([2.0])
        first = ci.next_positions()[0]  # 从 1.0 起步
        self.assertAlmostEqual(last, 1.0, places=6)
        self.assertGreaterEqual(first, last - 1e-9)
        self.assertAlmostEqual(first, 1.5, places=6)

    def test_midsegment_new_target_starts_from_published(self):
        # 未走完就来了新目标：以当前已发布位置为起点，保证连续。
        ci = CommandInterpolator(4)
        ci.mark_published([0.0])
        ci.set_target([1.0])
        published = ci.next_positions()[0]  # 0.25
        ci.set_target([0.0])
        nxt = ci.next_positions()[0]
        self.assertAlmostEqual(published, 0.25, places=6)
        # 新段从 0.25 朝 0 逼近，绝不跳变。
        self.assertLess(nxt, published + 1e-9)
        self.assertGreaterEqual(nxt, 0.0 - 1e-9)

    def test_length_mismatch_returns_target(self):
        ci = CommandInterpolator(4)
        ci.mark_published([0.0, 0.0])
        ci.set_target([1.0])  # 维度与起点不一致
        out = ci.next_positions()
        self.assertEqual(out, [1.0])

    def test_first_segment_without_baseline_no_motion(self):
        # 首次无已发布基线时，起点即目标，首段不产生位移（避免启动突跳）。
        ci = CommandInterpolator(4)
        ci.set_target([0.5])
        self.assertAlmostEqual(ci.next_positions()[0], 0.5, places=6)

    def test_reset_clears_state(self):
        ci = CommandInterpolator(4)
        ci.mark_published([0.0])
        ci.set_target([1.0])
        ci.reset()
        self.assertIsNone(ci.next_positions())


if __name__ == "__main__":
    unittest.main()
