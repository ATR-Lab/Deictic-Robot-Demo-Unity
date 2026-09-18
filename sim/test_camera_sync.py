import copy
import unittest
import numpy as np
from camera_sync import snapshot, synchronized


class Camera:
    def __init__(self,width=3,height=2):
        # Camera at (1,2,3), ROS optical axes aligned with base. USD camera
        # looks along -Z, so its row-vector view flips Y/Z and translates.
        view = np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[-1,2,3,1]],dtype=float)
        self.resolution = (width,height)
        self.frame = {"rgb":np.full((height,width,4),42,np.uint8),
                      "distance_to_image_plane":np.ones((height,width),np.float32),
                      "rendering_time":1.,
                      "rendering_frame":{"referenceTimeNumerator":60,"referenceTimeDenominator":60},
                      "CameraParams":{"cameraViewTransform":view,"metersPerSceneUnit":1.,"renderProductResolution":[width,height]}}
    def get_resolution(self):
        return self.resolution
    def get_current_frame(self,clone=False):
        return copy.deepcopy(self.frame) if clone else self.frame
    def get_rgba(self):
        raise AssertionError("Must not mix latest RGB with cached depth")
    def get_world_pose(self,**kwargs):
        raise AssertionError("Must not mix current pose with earlier captured image")


class CameraSyncTests(unittest.TestCase):
    def test_snapshot_uses_capture_pose_and_cached_rgb(self):
        camera = Camera()
        frame = snapshot(camera,True)
        np.testing.assert_allclose(frame.base_from_optical[:3,3],[1,2,3])
        np.testing.assert_allclose(frame.base_from_optical[:3,:3],np.eye(3))
        self.assertTrue(np.all(frame.rgb == 42))
        camera.frame["rgb"][:] = 0
        self.assertTrue(np.all(frame.rgb == 42))

    def test_reject_missing_capture_data(self):
        for key in ("rgb","CameraParams","rendering_frame","distance_to_image_plane"):
            camera = Camera()
            del camera.frame[key]
            self.assertIsNone(snapshot(camera,True))

    def test_reject_cross_frame_pair_and_invalid_view(self):
        one,two = Camera(),Camera()
        self.assertTrue(synchronized(snapshot(one),snapshot(two,True)))
        two.frame["rendering_frame"]["referenceTimeNumerator"] = 61
        self.assertFalse(synchronized(snapshot(one),snapshot(two,True)))
        two.frame["CameraParams"]["cameraViewTransform"][:] = 0
        self.assertIsNone(snapshot(two))

    def test_native_headset_size_can_differ_from_wrist(self):
        wrist,headset = Camera(),Camera(width=6,height=4)
        first,second = snapshot(wrist),snapshot(headset,True)
        self.assertTrue(synchronized(first,second))
        self.assertEqual(second.rgb.shape,(4,6,3))
        self.assertEqual(second.depth.shape,(4,6))
        headset.frame['distance_to_image_plane'] = np.ones((2,3),np.float32)
        self.assertIsNone(snapshot(headset,True))


if __name__ == "__main__":
    unittest.main()
