"""Fast model/trajectory invariants; run with python3 -m unittest discover -s sim."""
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from k1_model import ARM_JOINTS, LEFT_ARM_JOINTS, BOTH_ARM_JOINTS, URDF, Trajectory, fixed_arm_urdf, joint_limits, validate_positions, merge_arm_positions


class ModelTests(unittest.TestCase):
    def test_assets_complete_and_22_joints(self):
        self.assertEqual(len(joint_limits()), 22)
        for mesh in ET.parse(URDF).getroot().iter("mesh"):
            path = URDF.parent / mesh.attrib["filename"]
            self.assertTrue(path.is_file(), path)
            self.assertGreater(path.stat().st_size, 100)

    def test_only_both_arms_move_in_derived_model(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"fixed.urdf"
            fixed_arm_urdf(path)
            names = [j.attrib["name"] for j in ET.parse(path).getroot().findall("joint") if j.attrib["type"] != "fixed"]
            self.assertEqual(names, list(BOTH_ARM_JOINTS))

    def test_bimanual_mapping_and_right_only_preserves_left(self):
        values = [.1,.2,.3,.4,.1,.5,.3,.4]
        self.assertEqual(merge_arm_positions(list(reversed(BOTH_ARM_JOINTS)),list(reversed(values)),[0]*8), values)
        self.assertEqual(merge_arm_positions(ARM_JOINTS,[.2]*4,values),values[:4]+[.2]*4)
        with self.assertRaises(ValueError):
            merge_arm_positions(LEFT_ARM_JOINTS,[0]*4,values)

    def test_commands_are_name_mapped(self):
        values = [.1,.2,.3,.4]
        self.assertEqual(validate_positions(list(reversed(ARM_JOINTS)), list(reversed(values))), values)

    def test_bad_commands_rejected(self):
        for positions in ([0,0,0,math.nan], [0,0,0,5.], [0.,0.]):
            with self.assertRaises(ValueError):
                validate_positions(ARM_JOINTS, positions)
        with self.assertRaises(ValueError):
            validate_positions([ARM_JOINTS[0]]*4, [0]*4)

    def test_interpolation_and_hold(self):
        trajectory = Trajectory(ARM_JOINTS, [(1., [0,.5,0,0]), (2., [0,1.,0,0])], [0]*4)
        self.assertEqual(trajectory.sample(.5), [0,.25,0,0])
        self.assertEqual(trajectory.sample(5.), [0,1.,0,0])

    def test_bad_timing_rejected(self):
        for points in ([], [(1.,[0]*4),(.5,[0]*4)], [(math.inf,[0]*4)], [(-1.,[0]*4)], [(0.,[1]*4)]):
            with self.assertRaises(ValueError):
                Trajectory(ARM_JOINTS, points, [0]*4)


if __name__ == "__main__":
    unittest.main()
