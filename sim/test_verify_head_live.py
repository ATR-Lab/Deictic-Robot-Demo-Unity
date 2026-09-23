"""Pure validation checks for the opt-in live head/camera verifier."""
import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from head_control import orientation_targets
from verify_head_live import graph_ready, paired_digests, prepare_output, quaternion, rgb_digest, span


def message(data=b"\x01\x02\x03\x04\x05\x06", step=6, eye="left", stamp=100):
    return SimpleNamespace(encoding="rgb8", width=2, height=1, step=step, data=data,
                           header=SimpleNamespace(frame_id=eye, stamp=SimpleNamespace(sec=stamp, nanosec=0)))


class HeadLiveVerifierTests(unittest.TestCase):
    def test_graph_requires_one_identified_isaac_and_no_competing_commands(self):
        graph = dict(isaac_node_count=1, head_consumers=["k1_fixed_base_isaac"],
                     head_publishers=["synthetic_isaac_head_verifier"],
                     head_status_publishers=["k1_fixed_base_isaac"], joint_publishers=["k1_fixed_base_isaac"],
                     left_publishers=["k1_fixed_base_isaac"], right_publishers=["k1_fixed_base_isaac"])
        self.assertTrue(graph_ready(graph))
        for key in graph:
            changed = copy.deepcopy(graph)
            changed[key] = 2 if key == "isaac_node_count" else changed[key]+["other"]
            with self.subTest(key=key):
                self.assertFalse(graph_ready(changed))

    def test_hash_uses_actual_pixels_and_excludes_padding(self):
        packed = rgb_digest(message())
        padded = rgb_digest(message(data=b"\x01\x02\x03\x04\x05\x06\xff", step=7))
        changed = rgb_digest(message(data=b"\x01\x02\x03\x04\x05\x07"))
        self.assertEqual(packed["sha256"], padded["sha256"])
        self.assertNotEqual(packed["sha256"], changed["sha256"])
        self.assertEqual((packed["minimum"], packed["maximum"]), (1, 6))

    def test_invalid_image_shape_or_payload_rejected(self):
        for key, value in (("encoding", "rgba8"), ("width", 0), ("height", 481),
                           ("step", 5), ("step", 4097), ("data", b"")):
            invalid = message(); setattr(invalid, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                rgb_digest(invalid)

    def test_pair_requires_same_acquisition_and_distinct_eyes(self):
        left, right = rgb_digest(message()), rgb_digest(message(eye="right"))
        self.assertEqual(paired_digests(left, right)["stamp_ns"], 100_000_000_000)
        for key, value in (("stamp_ns", 101_000_000_000), ("frame_id", "left"), ("width", 3)):
            invalid = dict(right); invalid[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                paired_digests(left, invalid)

    def test_requested_moderate_targets_and_limit_probe(self):
        for yaw, pitch in ((0., 0.), (.25, -.1), (-.25, .1)):
            for actual, expected in zip(orientation_targets(quaternion(yaw, pitch)), (yaw, pitch)):
                self.assertAlmostEqual(actual, expected)
        self.assertEqual(orientation_targets(quaternion(1.4, 1.)), [1.012, .794])

    def test_hold_span_needs_three_independent_samples(self):
        with self.assertRaises(ValueError):
            span([[0, 0], [0, 0]])
        values = span([[.1, .2], [.11, .18], [.12, .21]])
        self.assertAlmostEqual(values[0], .02)
        self.assertAlmostEqual(values[1], .03)

    def test_output_preflight_creates_writable_destination_without_overwriting_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"nested"/"report.json"
            prepare_output(path)
            self.assertTrue(path.is_file())
            path.write_text("previous evidence")
            prepare_output(path)
            self.assertEqual(path.read_text(), "previous evidence")

    def test_output_preflight_surfaces_write_denial_before_ros_is_imported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"report.json"
            with mock.patch.object(Path, "open", side_effect=PermissionError("root-owned output")):
                with self.assertRaises(PermissionError):
                    prepare_output(path)


if __name__ == "__main__":
    unittest.main()
