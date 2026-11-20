"""Three-view projection coverage, seam overlap and acquisition contracts."""
import copy
from pathlib import Path
import unittest
import numpy as np
import yaml
from evtol_targeting.configuration import validate_config
from evtol_targeting.surround_geometry import optical_pose,intrinsics
from evtol_targeting.geometry import compensated_projection
from evtol_targeting.onnx_detector import decode
from evtol_simulation.surround_settings import camera_settings
from evtol_simulation.backend import check_camera_projection

ROOT=Path(__file__).resolve().parents[1]


class ThreeCamera(unittest.TestCase):
    def setUp(self):
        self.c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))
        self.s=self.c['surround']

    def visible(self,points):
        pose=(np.zeros(3),np.eye(3));body=(np.zeros(3),[0,0,0,1])
        return [set(compensated_projection(points,pose,pose,body,optical_pose(v),intrinsics(self.s),self.s['width'],self.s['height'])[1].point_indices.tolist()) for v in self.s['cameras']]

    def test_forward_horizontal_hemisphere_and_overlap_are_observed(self):
        az=np.radians(np.linspace(-90,90,721))
        points=np.concatenate([np.column_stack((10*np.cos(az)*np.cos(el),10*np.sin(az)*np.cos(el),np.full(len(az),10*np.sin(el)))) for el in np.radians([-10,0,10])])
        views=self.visible(points)
        self.assertEqual(set.union(*views),set(range(len(points))))
        for azimuth in (-30,30):
            a=np.radians(azimuth);point=np.array([[10*np.cos(a),10*np.sin(a),0]])
            self.assertEqual(sum(0 in ids for ids in self.visible(point)),2)

    def test_full_sphere_is_not_falsely_declared_covered(self):
        views=self.visible(np.array([[0,0,10],[0,0,-10],[-10,0,0]]))
        self.assertFalse(set.union(*views))

    def test_portrait_upper_coverage_and_lower_cruise_layer(self):
        angles=np.radians(np.linspace(-90,90,721))
        points=np.concatenate([np.column_stack((10*np.cos(angles)*np.cos(el),10*np.sin(angles)*np.cos(el),np.full(len(angles),10*np.sin(el)))) for el in np.radians([-20,45])])
        self.assertEqual(set.union(*self.visible(points)),set(range(len(points))))

    def test_three_camera_detection_valid_and_semantic_layout_rejected(self):
        validate_config(self.c)
        c=copy.deepcopy(self.c);c['model'].update(task='semantic',backend='torch')
        with self.assertRaisesRegex(ValueError,'six cameras'):validate_config(c)

    def test_truth_is_only_enabled_for_primary_diagnostic_or_capture(self):
        for i,view in enumerate(self.s['cameras']):
            settings=camera_settings(self.s,view)
            self.assertEqual([v['ImageType'] for v in settings['CaptureSettings']],[0,5] if i==0 else [0])
        s=copy.deepcopy(self.s);s['record_truth']=True
        for view in s['cameras']:
            self.assertEqual([v['ImageType'] for v in camera_settings(s,view)['CaptureSettings']],[0,5])

    def test_three_onnx_outputs_retain_individual_view_indices(self):
        rows=np.array([[[1,2,20,30,.9,0]]]*3)
        boxes=decode(rows,[(1.,0,0,40,40)]*3,.3,128)
        self.assertEqual([b['view_index'] for b in boxes],[0,1,2])

    def test_portrait_airsim_metadata_keeps_strict_fov_check(self):
        f=1/np.tan(np.radians(35));aspect=1536/2048
        camera=dict(width=1536,height=2048,horizontal_fov_deg=70)
        for scales in ([f,-f*aspect],[f/aspect,-f]):
            matrix=np.eye(4);matrix[0,1],matrix[1,2]=scales
            check_camera_projection(dict(proj_mat=dict(matrix=matrix)),camera)
            with self.assertRaises(ValueError):
                check_camera_projection(dict(proj_mat=dict(matrix=matrix)),dict(camera,horizontal_fov_deg=75))


if __name__=='__main__':unittest.main()
