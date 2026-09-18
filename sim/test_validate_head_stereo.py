import copy
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from validate_head_stereo import image_statistics, pair_metrics, pixels, relay_metrics


def message(rgb, frame='left', stamp=123):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=1),frame_id=frame),
                           width=rgb.shape[1],height=rgb.shape[0],step=rgb.shape[1]*3,
                           encoding='rgb8',data=rgb.tobytes())


def info(image,right=False):
    return SimpleNamespace(header=copy.deepcopy(image.header),width=image.width,height=image.height,
                           k=[320.,0,320,0,320,240,0,0,1],
                           p=[320.,0,320,-20.48 if right else 0,0,320,240,0,0,0,1,0])


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.rgb = np.arange(6*8*3,dtype=np.uint8).reshape(6,8,3)
        self.left = message(self.rgb)
        self.right = message(np.flip(self.rgb,axis=1).copy(),'right')

    def test_exact_pair_reports_pixels_and_assumed_baseline(self):
        result = pair_metrics(self.left,self.right,info(self.left),info(self.right,True))
        self.assertAlmostEqual(result['inferred_simulation_baseline_m'],.064)
        self.assertGreater(result['mean_absolute_eye_difference'],0)
        self.assertGreater(result['left']['spatial_gray_std'],1)

    def test_reject_header_calibration_and_payload_mismatch(self):
        other = copy.deepcopy(self.right)
        other.header.stamp.nanosec += 1
        with self.assertRaises(ValueError): pair_metrics(self.left,other,info(self.left),info(other,True))
        calibration = info(self.left); calibration.width += 1
        with self.assertRaises(ValueError): pair_metrics(self.left,self.right,calibration,info(self.right,True))
        other.data = other.data[:-1]
        with self.assertRaises(ValueError): pixels(other)

    def test_composite_requires_exact_stamp_and_left_right_pixel_order(self):
        resized = [cv2.resize(pixels(eye),(4,3),interpolation=cv2.INTER_AREA) for eye in (self.left,self.right)]
        relay = message(np.concatenate(resized,axis=1),'k1_head_stereo_optical')
        self.assertTrue(relay_metrics(self.left,self.right,relay)['exact_source_halves'])
        swapped = message(np.concatenate(resized[::-1],axis=1),'k1_head_stereo_optical')
        self.assertFalse(relay_metrics(self.left,self.right,swapped)['exact_source_halves'])
        relay.header.stamp.sec += 1
        with self.assertRaises(ValueError): relay_metrics(self.left,self.right,relay)

    def test_blank_and_uniform_color_have_no_spatial_signal(self):
        black = image_statistics(np.zeros((6,8,3),np.uint8))
        self.assertEqual(black['black_fraction'],1)
        self.assertEqual(black['spatial_gray_std'],0)
        color = np.full((6,8,3),[20,40,150],np.uint8)
        self.assertGreater(image_statistics(color)['std'],1)
        self.assertEqual(image_statistics(color)['spatial_gray_std'],0)


if __name__ == '__main__':
    unittest.main()
