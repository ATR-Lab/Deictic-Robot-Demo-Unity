import copy
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
import struct
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

from camera_sync import CameraSnapshot
from head_stereo import HeadStereoDisplay, StereoSchedule, stereo_mount, rigid_link_prim, frame_diagnostic
from k1_model import URDF


def frame(reference=60, width=4, height=3):
    return CameraSnapshot(np.full((height,width,3),42,np.uint8), None,
                          np.eye(4), reference/60, (reference,60))


class HeadStereoGeometryTests(unittest.TestCase):
    def test_importer_duplicate_names_select_only_rigid_link(self):
        def prim(path,rigid):
            return types.SimpleNamespace(GetName=lambda:'aahead_pitch_link',
                                         GetPath=lambda:path,HasAPI=lambda api:rigid)
        rigid = prim('/K1/aahead_pitch_link',True)
        stage = types.SimpleNamespace(Traverse=lambda:[
            prim('/K1/aahead_pitch_link/visuals/aahead_pitch_link',False),
            rigid,prim('/K1/aahead_pitch_link/collisions/aahead_pitch_link',False)])
        self.assertIs(rigid_link_prim(stage,'aahead_pitch_link',object()),rigid)
        for candidates in ([],[rigid,prim('/Other/aahead_pitch_link',True)]):
            with self.assertRaises(RuntimeError):
                rigid_link_prim(types.SimpleNamespace(Traverse=lambda:candidates),'aahead_pitch_link',object())

    def test_vendor_optical_orientation_and_left_right_baseline(self):
        spec = stereo_mount()
        rotation = np.array(spec['rotation_parent_from_optical'])
        # Head parent uses robot +X forward,+Y left,+Z up. Optical axes differ.
        np.testing.assert_allclose(rotation[:,2], [1,0,0], atol=1e-6)
        np.testing.assert_allclose(rotation[:,0], [0,-1,0], atol=1e-6)
        np.testing.assert_allclose(rotation[:,1], [0,0,-1], atol=1e-6)
        left = np.array(spec['eyes']['left']['position_parent'])
        right = np.array(spec['eyes']['right']['position_parent'])
        np.testing.assert_allclose(right-left, rotation[:,0]*.064, atol=1e-10)
        self.assertGreater(left[1], right[1])
        self.assertEqual(spec['parent_link'], 'aahead_pitch_link')
        self.assertTrue(spec['simulation_only'])

    def test_optical_origins_are_in_front_of_entire_vendor_head_mesh(self):
        data = (URDF.parent/'meshes/Head_2.STL').read_bytes()
        count = struct.unpack_from('<I',data,80)[0]
        dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        vertices = np.frombuffer(data,dtype=dtype,count=count,offset=84)['vertices']
        maximum_x = float(vertices[:,:,0].max())
        for eye in stereo_mount()['eyes'].values():
            self.assertGreater(eye['position_parent'][0]-maximum_x,.016)

    def test_simulation_baseline_is_configurable_and_rejects_invalid_values(self):
        for value in (0,-.01,float('nan'),float('inf'),.21):
            with self.assertRaises(ValueError): stereo_mount(value)
        spec = stereo_mount(.08,320)
        self.assertEqual(spec['resolution'],[320,240])
        left,right = [np.array(spec['eyes'][side]['position_parent']) for side in ('left','right')]
        self.assertAlmostEqual(np.linalg.norm(right-left),.08)
        with self.assertRaises(ValueError): stereo_mount(width=1280)


class HeadStereoScheduleTests(unittest.TestCase):
    def test_no_demand_pending_capture_timeout_and_backoff(self):
        schedule = StereoSchedule()
        self.assertFalse(schedule.request(1.,False))
        self.assertTrue(schedule.request(1.,True))
        self.assertTrue(schedule.request(1.19,True))
        self.assertEqual(schedule.requests,1)
        self.assertTrue(schedule.request(2.99,True))
        self.assertFalse(schedule.request(3.01,True))
        self.assertEqual(schedule.timeouts,1)
        self.assertFalse(schedule.request(3.10,True))
        self.assertTrue(schedule.request(3.22,True))
        self.assertEqual(schedule.requests,2)
        self.assertFalse(schedule.request(3.23,False))
        self.assertFalse(schedule.pending)

    def test_request_watermark_excludes_pre_request_cached_frames(self):
        schedule = StereoSchedule()
        self.assertTrue(schedule.request(1.,True,Fraction(61,60)))
        self.assertFalse(schedule.accept(frame(60),frame(60),1.1))
        self.assertFalse(schedule.accept(frame(61),frame(61),1.1))
        self.assertTrue(schedule.accept(frame(62),frame(62),1.1))
        self.assertFalse(schedule.pending)

    def test_reject_mismatched_missing_and_unequal_size_frames(self):
        schedule = StereoSchedule()
        self.assertFalse(schedule.accept(None,frame(),1.))
        self.assertFalse(schedule.accept(frame(60),frame(61),1.))
        self.assertFalse(schedule.accept(frame(),replace(frame(),rendering_time=2.),1.))
        self.assertFalse(schedule.accept(frame(),frame(width=8),1.))
        self.assertFalse(schedule.accept(frame(width=1280),frame(width=1280),1.))
        self.assertTrue(schedule.accept(frame(),frame(),1.))

    def test_duplicate_old_frames_and_publication_rate_rejected_on_resume(self):
        schedule = StereoSchedule()
        self.assertTrue(schedule.accept(frame(),frame(),1.))
        self.assertFalse(schedule.accept(frame(61),frame(61),1.1))
        self.assertTrue(schedule.accept(frame(61),frame(61),1.21))
        self.assertFalse(schedule.request(2.,False))
        self.assertTrue(schedule.request(10.,True))
        self.assertFalse(schedule.accept(frame(61),frame(61),10.))
        self.assertFalse(schedule.accept(frame(),frame(),10.))
        self.assertTrue(schedule.accept(frame(62),frame(62),10.))


class Publisher:
    def __init__(self): self.demand,self.messages = 0,[]
    def get_subscription_count(self): return self.demand
    def publish(self,message): self.messages.append(copy.deepcopy(message))


class Product:
    def __init__(self): self.hydra_texture,self.enabled = self,False
    def set_updates_enabled(self,value): self.enabled = value


class Message:
    def __init__(self): self.header = types.SimpleNamespace(stamp=None,frame_id='')


class HeadStereoPublisherTests(unittest.TestCase):
    def make_display(self):
        display = HeadStereoDisplay.__new__(HeadStereoDisplay)
        display.spec = stereo_mount()
        display.products = {side:Product() for side in ('left','right')}
        display.publishers = {side:(Publisher(),Publisher()) for side in ('left','right')}
        display.cameras = {side:types.SimpleNamespace(
            get_current_frame=lambda clone=False:{'rendering_frame':0,'rendering_time':0},
            is_paused=lambda:False,get_render_product_path=lambda:side) for side in ('left','right')}
        display.clock_ns = 123_100_000_456
        clock = types.SimpleNamespace(now=lambda:types.SimpleNamespace(
            nanoseconds=display.clock_ns,
            to_msg=lambda:types.SimpleNamespace(sec=display.clock_ns//1_000_000_000,
                                               nanosec=display.clock_ns%1_000_000_000)))
        display.node = types.SimpleNamespace(get_clock=lambda:clock)
        display.intrinsics = lambda camera: np.array([[320.,0,320],[0,320,240],[0,0,1]])
        display.schedule,display.requested,display.saved = StereoSchedule(),False,True
        display.published_pairs = 0
        display.reference_times = {}
        display.render_opportunities,display.requested_renders = 0,0
        display.next_diagnostic = float('inf')
        display.subscriber_counts,display.last_decision = {},''
        return display

    def test_render_products_wait_for_deferred_callback_then_disable_without_demand(self):
        display = self.make_display()
        display.prepare_render(1.)
        self.assertFalse(any(product.enabled for product in display.products.values()))
        display.publishers['right'][0].demand = 1
        display.prepare_render(1.)
        self.assertTrue(all(product.enabled for product in display.products.values()))
        with patch('head_stereo.snapshot',return_value=None): display.finish_render(1.)
        self.assertTrue(all(product.enabled for product in display.products.values()))
        display.prepare_render(1.1)
        self.assertTrue(display.schedule.pending)
        self.assertEqual(display.schedule.requests,1)
        display.publishers['right'][0].demand = 0
        display.prepare_render(1.11)
        self.assertFalse(any(product.enabled for product in display.products.values()))
        self.assertEqual(display.published_pairs,0)

    def test_reference_history_bounded_and_first_observation_not_refreshed(self):
        display = self.make_display()
        raw={'rendering_frame':{'referenceTimeNumerator':60,'referenceTimeDenominator':60},'rendering_time':1.}
        display.observe_render_reference(raw,100,1.)
        display.observe_render_reference(raw,200,2.)
        self.assertEqual(display.reference_times[Fraction(1)],(100,1.,1.))
        for number in range(61,140):
            raw={'rendering_frame':{'referenceTimeNumerator':number,'referenceTimeDenominator':60},'rendering_time':number/60}
            display.observe_render_reference(raw,number,number/60)
        self.assertEqual(len(display.reference_times),64)
        self.assertNotIn(Fraction(1),display.reference_times)

    def test_unknown_or_old_capture_never_gets_a_new_timestamp(self):
        display = self.make_display()
        display.publishers['left'][0].demand = 1
        display.prepare_render(1.)
        with patch('head_stereo.snapshot',return_value=frame()):
            display.finish_render(1.1)
            self.assertEqual(display.last_decision,'waiting_for_known_render_reference')
            raw={'rendering_frame':{'referenceTimeNumerator':60,'referenceTimeDenominator':60},'rendering_time':1.}
            display.observe_render_reference(raw,122_000_000_456,0.)
            display.finish_render(1.2)
            self.assertEqual(display.last_decision,'capture_too_old')
        self.assertEqual(display.published_pairs,0)

    def test_pair_and_intrinsics_share_stamp_with_distinct_frames(self):
        display = self.make_display()
        display.publishers['left'][0].demand = 1
        display.prepare_render(1.)
        display.observe_render_reference(
            {'rendering_frame':{'referenceTimeNumerator':60,'referenceTimeDenominator':60},'rendering_time':1.},
            123_000_000_456,1.)
        msg_module = types.ModuleType('sensor_msgs.msg')
        msg_module.Image,msg_module.CameraInfo = Message,Message
        with patch.dict(sys.modules,{'sensor_msgs.msg':msg_module}), \
             patch('head_stereo.snapshot',return_value=frame(width=640,height=480)):
            display.finish_render(1.01)
        self.assertEqual(display.published_pairs,1)
        left,right = [display.publishers[side][0].messages[0] for side in ('left','right')]
        self.assertEqual(left.header.stamp,right.header.stamp)
        self.assertEqual(left.header.stamp.sec,123)
        self.assertEqual(left.header.stamp.nanosec,456)
        self.assertNotEqual(left.header.stamp.nanosec,display.clock_ns%1_000_000_000)
        self.assertFalse(any(product.enabled for product in display.products.values()))
        self.assertNotEqual(left.header.frame_id,right.header.frame_id)
        self.assertEqual((left.width,left.height,left.step,left.encoding),(640,480,1920,'rgb8'))
        for side in ('left','right'):
            image,info = [publisher.messages[0] for publisher in display.publishers[side]]
            self.assertEqual(image.header,info.header)
            self.assertEqual(info.p[3],0. if side == 'left' else -320*.064)

    def test_diagnostics_contain_metadata_not_pixel_arrays(self):
        raw={'rgb':np.zeros((480,640,4),np.uint8),'rendering_time':1.,
             'rendering_frame':{'referenceTimeNumerator':60,'referenceTimeDenominator':60},
             'CameraParams':{'renderProductResolution':[640,480],'cameraViewTransform':np.eye(4),'metersPerSceneUnit':1.}}
        camera=types.SimpleNamespace(get_current_frame=lambda clone=False:raw,
                                     is_paused=lambda:False,get_render_product_path=lambda:'/Render/HeadLeft')
        result=frame_diagnostic(camera,False)
        self.assertEqual(result['rgb_shape'],[480,640,4])
        self.assertEqual(result['camera_params_resolution'],[640,480])
        self.assertNotIn('rgb',result)
        self.assertNotIn('cameraViewTransform',result)


if __name__ == '__main__':
    unittest.main()
