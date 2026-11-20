"""Analytic mechanics, reference geometry, causal ordering and independent truth audit."""
import copy
import unittest
import numpy as np
from evtol_localization.eskf import ESKF,expq,rotation
from evtol_localization.stream import FusionStream
from evtol_localization.options import options,fault_at
from evtol_localization.geodesy import GeoReference
from evtol_localization.evaluation import ErrorAudit

T=1_700_000_000_000_000_000
def imu(q=None):return dict(accel=np.array([0.,0.,9.80665]),gyro=np.zeros(3),q=np.array([0.,0.,0.,1.]) if q is None else q)
def fix(p=None):return dict(position=np.zeros(3) if p is None else np.asarray(p),covariance=np.eye(3)*.25,valid=True)


class FilterTests(unittest.TestCase):
    def initialized(self,**kwargs):
        f=ESKF(kwargs);f.initialize(T,[0.,0.,0.],[0.,0.,0.,1.],np.eye(3)*.25);return f

    def test_ideal_static_and_positive_covariance(self):
        f=self.initialized()
        for i in range(1,201):f.propagate(T+i*10_000_000,[0,0,9.80665],[0,0,0])
        np.testing.assert_allclose(f.p,0,atol=1e-10);np.testing.assert_allclose(f.v,0,atol=1e-10)
        self.assertGreater(np.linalg.eigvalsh(f.P).min(),0)
        self.assertGreater(f.P[0,3],0)

    def test_north_acceleration_and_body_twist_rotation(self):
        f=self.initialized();f.q=expq([0,0,np.pi/2])
        for i in range(1,101):f.propagate(T+i*10_000_000,[1,0,9.80665],[0,0,0])
        np.testing.assert_allclose(f.p,[0,.5,0],atol=1e-8);np.testing.assert_allclose(f.v,[0,1,0],atol=1e-8)
        velocity,_,pc,tc=f.body_output([0,0,0]);np.testing.assert_allclose(velocity,[1,0,0],atol=1e-8)
        self.assertEqual(pc.shape,(6,6));self.assertGreaterEqual(np.linalg.eigvalsh(tc).min(),0)

    def test_gnss_and_range_outliers_do_not_modify_state(self):
        f=self.initialized();p=f.p.copy();P=f.P.copy()
        self.assertFalse(f.gnss([1000,0,0],np.eye(3))['accepted'])
        self.assertFalse(f.range([10,0,0],1000,.15)['accepted'])
        np.testing.assert_array_equal(f.p,p);np.testing.assert_array_equal(f.P,P)

    def test_single_anchor_unconstrained_axes_keep_uncertainty(self):
        f=self.initialized();before=np.diag(f.P).copy()
        self.assertTrue(f.range([10,0,0],10,.15)['accepted'])
        self.assertLess(f.P[0,0],before[0]);self.assertAlmostEqual(f.P[1,1],before[1]);self.assertAlmostEqual(f.P[2,2],before[2])

    def test_antenna_lever_arm_not_double_shifted(self):
        f=self.initialized(fiveg_lever_flu_m=[1.,0.,0.]);f.p=np.array([2.,3.,4.])
        self.assertAlmostEqual(f.range([10,3,4],7.,.2)['jump_m'],0.)

    def test_unknown_orientation_and_imu_gap_rejected(self):
        f=self.initialized()
        with self.assertRaises(ValueError):f.propagate(T+int(1e9),[0,0,9.8],[0,0,0])
        with self.assertRaises(ValueError):f.initialize(T,[0,0,0],[0,0,0,0],np.eye(3))


class StreamTests(unittest.TestCase):
    def run_stream(self,late=False,config=None):
        s=FusionStream(config)
        packets=[]
        for i in range(201):
            t=T+i*10_000_000;packets.append(('imu',t,imu()))
            if i%10==0:packets.append(('gnss',t,fix()))
        if late:packets=packets[1::2]+packets[::2]
        for kind,t,data in packets:s.push(kind,t,data)
        s.drain(T+int(2.24e9),T);return s

    def test_bounded_order_matches_inorder_and_no_future_processing(self):
        a=self.run_stream();b=self.run_stream(True)
        np.testing.assert_allclose(a.filter.p,b.filter.p);np.testing.assert_allclose(a.filter.P,b.filter.P)
        self.assertEqual(a.filter.t,T+int(2e9));self.assertGreater(a.stats['used_gnss'],10)
        a.push('imu',T+int(3e9),imu());a.drain(T+int(2.9e9),T)
        self.assertEqual(a.filter.t,T+int(2e9))

    def test_duplicates_late_and_reset_clear_everything(self):
        s=self.run_stream();self.assertFalse(s.push('imu',T+int(2e9),imu()))
        self.assertFalse(s.push('gnss',T-1,fix()));self.assertIn('late_outside_reorder',s.last_rejection)
        s.reset('session_change');self.assertIsNone(s.filter.t);self.assertEqual(len(s.seen),0);self.assertEqual(len(s.fix_history),0)

    def test_gnss_outage_and_recovery_only_affect_estimator(self):
        o=options();o['gnss_fault'].update(enabled=True,segments=[dict(start_s=.8,end_s=1.4,mode='dropout',bias_enu_m=[0,0,0])])
        s=self.run_stream(config=o)
        self.assertEqual(s.stats['gnss_injected_dropout'],6)
        self.assertTrue(s.recoveries[0]['accepted']);self.assertEqual(s.recoveries[0]['stamp_ns'],T+int(1.4e9))
        self.assertAlmostEqual(s.recoveries[0]['jump_m'],0.)

    def test_per_station_validation_and_no_simulator_labels(self):
        s=self.run_stream(config=dict(mode='gnss_imu_5g'));t=s.filter.t
        link=dict(id='A',anchor=[10,0,0],valid=True,range=10.,sigma=.15,rsrp=-50.)
        s.fiveg(t,[link]);self.assertEqual(s.anchors['A']['accepted'],1)
        s.fiveg(t,[dict(link,id='B',rsrp=-130),dict(link,id='C',valid=False),dict(link,id='D',sigma=-1),dict(link,id='E',range=1000)])
        for name in ('B','C','D','E'):self.assertEqual(s.anchors[name]['rejected'],1)
        a=self.run_stream(config=dict(mode='gnss_imu_5g'));b=self.run_stream(config=dict(mode='gnss_imu_5g'))
        a.fiveg(t,[dict(link,is_los=True,true_range=10)]);b.fiveg(t,[dict(link,is_los=False,true_range=100000)])
        np.testing.assert_array_equal(a.filter.P,b.filter.P)

    def test_waits_for_orientation_and_initialization_window(self):
        s=FusionStream();d=imu();d['q']=None
        s.push('imu',T,d);s.push('gnss',T,fix());s.drain(T+int(.3e9),T)
        self.assertIsNone(s.filter.t);self.assertIn('waiting_imu_orientation',s.last_rejection)

    def test_static_bias_identification(self):
        s=self.run_stream();self.assertEqual(s.stats['initializations'],1)
        self.assertIn('initialized_stationary',[v['reason'] for v in s.corrections])


class ReferenceAndAuditTests(unittest.TestCase):
    def test_nonzero_session_origin_anchor_invariance(self):
        origin=np.array([31.,-42.,12.]);p=np.array([45.,9.,34.]);anchor=np.array([-60.,70.,20.])
        geo=GeoReference(47.641468,-122.140165,122.,origin)
        np.testing.assert_allclose(geo.position(47.641468,-122.140165,122.),-origin)
        self.assertAlmostEqual(np.linalg.norm(p-anchor),np.linalg.norm(p-origin-geo.anchor(anchor)))

    def test_truth_interpolation_and_coverage_no_alignment_fit(self):
        a=ErrorAudit();q=np.array([0.,0.,0.,1.])
        a.estimate(T,np.array([1,0,0]),q);a.estimate(T+int(.1e9),np.array([1,1,0]),q)
        a.truth(T+int(.05e9),np.array([0,.5,0]),q);a.truth(T+int(.2e9),np.array([0,2,0]),q)
        a.drain(T+int(.3e9),lambda t:'normal')
        self.assertAlmostEqual(a.summary()['rmse_3d_m'],1.);self.assertEqual(a.summary()['coverage'],.5)

    def test_schedule_normal_and_multiple_outages(self):
        o=options()['gnss_fault'];o['enabled']=True
        self.assertEqual(fault_at(o,39.9)[0],'normal');self.assertEqual(fault_at(o,40)[0],'dropout');self.assertEqual(fault_at(o,60)[0],'normal')
        o['segments'].append(dict(start_s=59,end_s=70,mode='normal',bias_enu_m=[0,0,0]))
        with self.assertRaises(ValueError):options(dict(gnss_fault=o))


if __name__=='__main__':unittest.main()
