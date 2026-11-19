import copy
from pathlib import Path
import unittest
import numpy as np
import yaml
from evtol_targeting.surround_geometry import optical_pose,intrinsics,lidar_settings,merge_views
from evtol_targeting.geometry import compensated_projection
from evtol_targeting.clustering import extract_visible_targets
from evtol_targeting.configuration import validate_config
from evtol_targeting.projection import ProjectionResult

ROOT=Path(__file__).resolve().parents[1]


class CubeCoverage(unittest.TestCase):
    def setUp(self):
        self.c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'));self.s=self.c['surround']

    def test_six_views_cover_every_direction(self):
        rng=np.random.default_rng(17);points=rng.normal(size=(20000,3));points*=10/np.linalg.norm(points,axis=1)[:,None]
        pose=(np.zeros(3),np.eye(3));body=(np.zeros(3),[0,0,0,1]);seen=set()
        for v in self.s['cameras']:
            _,p=compensated_projection(points,pose,pose,body,optical_pose(v),intrinsics(self.s),self.s['width'],self.s['height'])
            seen.update(p.point_indices.tolist())
        self.assertEqual(len(seen),len(points))

    def test_side_back_and_vertical_detections_require_both_rgb_class_and_depth(self):
        pose=(np.zeros(3),np.eye(3));body=(np.zeros(3),[0,0,0,1]);w,h=self.s['width'],self.s['height']
        for v,direction in zip(self.s['cameras'],[[1,0,0],[0,-1,0],[-1,0,0],[0,1,0],[0,0,1],[0,0,-1]]):
            points=np.asarray(direction)*10+np.array([[.01,.01,.01],[-.01,-.01,-.01],[.02,-.02,.02]])
            world,p=compensated_projection(points,pose,pose,body,optical_pose(v),intrinsics(self.s),w,h)
            self.assertEqual(len(p.point_indices),3)
            mask=np.zeros((h,w),np.int32)
            for u,y in p.pixels_uv:mask[max(0,y-2):y+3,max(0,u-2):u+3]=6
            result=extract_visible_targets(world,p,mask,np.ones_like(mask),[6],min_lidar_points=2,min_mask_pixels=6)
            self.assertEqual(len(result),1,v['name'])
            mask[:]=0
            self.assertFalse(extract_visible_targets(world,p,mask,np.ones_like(mask),[6],min_lidar_points=2))

    def test_invalid_surround_budget_rejected(self):
        for field,value in [('max_age_s',2),('horizontal_fov_deg',70),('width',0)]:
            c=copy.deepcopy(self.c);c['surround'][field]=value
            with self.assertRaises(ValueError):validate_config(c)

    def test_moving_depth_needs_visual_class_within_bounded_motion(self):
        mask=np.zeros((40,40),np.int32);mask[18:22,18:22]=6
        points=np.array([[10,0,0],[10,.02,0],[10,0,.02]])
        projection=ProjectionResult(np.arange(3),np.array([[24,19],[24,20],[24,21]]),np.full(3,10.))
        def detect(radius):
            return extract_visible_targets(points,projection,mask,np.ones_like(mask),[6],
                min_mask_pixels=6,pixel_motion_radius=np.full(3,radius))
        self.assertFalse(detect(2.))
        self.assertEqual(len(detect(4.)),1)
        mask[:]=0
        self.assertFalse(detect(12.))


if __name__=='__main__':unittest.main()
