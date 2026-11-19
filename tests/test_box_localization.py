import copy
from pathlib import Path
import unittest
import numpy as np
import yaml
from evtol_targeting.box_localization import normalize_detections,box_rays,localize_boxes
from evtol_targeting.projection import ProjectionResult
from evtol_targeting.configuration import validate_config
from evtol_console.processes import node_specs

ROOT=Path(__file__).resolve().parents[1]


class BoxLocalization(unittest.TestCase):
    def setUp(self):
        self.model=dict(class_map={0:6},confidence=.25,max_det=128)
        self.box=dict(view_index=0,source_class_id=0,class_id=6,confidence=.9,xyxy=[40,40,60,60])
        self.options=dict(cluster_radius_m=.8,min_lidar_points=3,max_drone_extent_m=4.)

    def test_classes_coordinates_and_threshold_are_explicit(self):
        raw=[dict(self.box,xyxy=[-2,40,60,110]),dict(self.box,confidence=.2),
            dict(self.box,source_class_id=7),dict(self.box,xyxy=[50,40,49,50]),
            dict(self.box,view_index=6),dict(self.box,xyxy=[float('nan'),0,1,1])]
        result=normalize_detections(raw,self.model,100,100,6)
        self.assertEqual(len(result),1);self.assertEqual(result[0]['class_id'],6)
        self.assertEqual(result[0]['xyxy'],[0,40,60,100])

    def test_roi_rays_cover_each_instance_with_finite_strict_budget(self):
        boxes=[dict(self.box,xyxy=[x,10,x+2,12]) for x in range(0,96,3)]
        k=np.array([[50,0,50],[0,50,50],[0,0,1]])
        rays=np.asarray(box_rays(boxes,k,100,100,budget=640))
        self.assertLessEqual(len(rays),640);self.assertTrue(np.isfinite(rays).all())
        pixels=rays*50+50
        for b in boxes:
            center=(np.array(b['xyxy'][:2])+b['xyxy'][2:])/2
            self.assertLess(np.linalg.norm(pixels-center,axis=1).min(),1e-8)

    def test_real_depth_required_and_points_outside_box_do_not_localize(self):
        world=np.array([[5,0,10],[5,.1,10],[5,0,10.1]])
        outside=ProjectionResult(np.arange(3),np.array([[10,10],[11,10],[10,11]]),np.full(3,5.))
        self.assertFalse(localize_boxes(world,outside,[self.box],self.options)[0])
        empty=ProjectionResult(np.array([],int),np.empty((0,2),int),np.array([]))
        self.assertFalse(localize_boxes(world,empty,[self.box],self.options)[0])

    def test_compact_foreground_selected_instead_of_background_average(self):
        world=np.array([[5,0,10],[5,.1,10],[5,0,10.1],[20,0,10],[20,.1,10],[20,0,10.1]])
        projection=ProjectionResult(np.arange(6),np.array([[49,49],[50,49],[49,50],[54,54],[55,54],[54,55]]),np.array([5]*3+[20]*3))
        found,_=localize_boxes(world,projection,[self.box],self.options)
        self.assertEqual(len(found),1);self.assertLess(found[0].position_body_m[0],6)
        self.assertEqual(found[0].class_id,6)

    def test_overlapping_boxes_cannot_reuse_the_same_lidar_points(self):
        world=np.array([[5,0,10],[5,.1,10],[5,0,10.1]])
        projection=ProjectionResult(np.arange(3),np.array([[49,49],[50,49],[49,50]]),np.full(3,5.))
        found,_=localize_boxes(world,projection,[self.box,dict(self.box,confidence=.8)],self.options)
        self.assertEqual(len(found),1)

    def test_bounded_motion_margin_ground_filter_and_sparse_depth(self):
        world=np.array([[5,0,10],[5,.1,10],[5,0,10.1]])
        projection=ProjectionResult(np.arange(3),np.array([[62,49],[62,50],[62,51]]),np.full(3,5.))
        self.assertFalse(localize_boxes(world,projection,[self.box],self.options)[0])
        self.assertEqual(len(localize_boxes(world,projection,[self.box],self.options,np.full(3,3.))[0]),1)
        ground=world.copy();ground[:,2]=0
        self.assertFalse(localize_boxes(ground,projection,[self.box],self.options,np.full(3,3.),ground_z=.15)[0])
        sparse=ProjectionResult(np.arange(2),np.array([[50,50],[51,50]]),np.full(2,5.))
        self.assertFalse(localize_boxes(world,sparse,[self.box],self.options)[0])

    def test_detect_stack_never_launches_semantic_model_even_with_diagnostic_flag(self):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))
        c['model']['task']='detect';c['surround']['front_diagnostic_inference']=True
        modules=[m for _,m,_ in node_specs(c,'config','sim',dict(execute=False,actors=False,evaluate=True))]
        self.assertIn('evtol_targeting.detection_fusion',modules)
        self.assertIn('evtol_targeting.surround_vision',modules)
        self.assertNotIn('evtol_targeting.vision_node',modules)
        self.assertNotIn('evtol_targeting.surround_fusion',modules)
        self.assertNotIn('evtol_targeting.localizer_node',modules)

    def test_detection_configuration_rejects_wrong_mapping_and_threshold(self):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))
        validate_config(c)
        for key,value in [('confidence',0),('class_map',{0:3}),('max_det',1000)]:
            invalid=copy.deepcopy(c);invalid['model'][key]=value
            with self.assertRaises(ValueError):validate_config(invalid)


if __name__=='__main__':unittest.main()
