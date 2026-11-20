"""Private-DDS integration with analytic sensor inputs, not AirSim accuracy evidence."""
import copy
import json
import tempfile
from pathlib import Path
import time
import unittest
import numpy as np
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix,Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from evtol_perception_interfaces.msg import StateStatus
from evtol_sim_interfaces.msg import FiveGObservations,FiveGLink
from evtol_targeting.common import header,stamp,put,LATCHED
from evtol_targeting.state_node import StateNode
from evtol_localization.node import LocalizationNode,SENSOR_QOS
from evtol_localization.evaluation_node import EvaluationNode
from evtol_localization.options import options
from evtol_simulation.gnss_model import geodetic_to_ecef,enu_to_ecef_matrix


def inverse_geo(enu):
    x,y,z=geodetic_to_ecef(47.641468,-122.140165,122.)+enu_to_ecef_matrix(47.641468,-122.140165)@enu
    a=6378137.;e2=.0066943799901413165;p=np.hypot(x,y);lat=np.arctan2(z,p*(1-e2))
    for _ in range(8):
        n=a/np.sqrt(1-e2*np.sin(lat)**2);alt=p/np.cos(lat)-n;lat=np.arctan2(z,p*(1-e2*n/(n+alt)))
    return np.degrees(lat),np.degrees(np.arctan2(y,x)),float(alt)


class RosContracts(unittest.TestCase):
    def test_fused_forwarding_error_outage_recovery_and_timeout(self):
        rclpy.init(domain_id=229)
        temporary=tempfile.TemporaryDirectory();root=Path(__file__).resolve().parents[1]
        c=yaml.safe_load((root/'config/targeting.yaml').read_text(encoding='utf8'))
        c['_root']=Path(temporary.name);c['state'].update(source='fused',origin_file='origin.json')
        c['airsim']['port']=41599;c['localization']=options(dict(mode='gnss_imu_5g'))
        c['localization']['gnss_fault'].update(enabled=True,time_base='session',segments=[dict(start_s=2.,end_s=4.,mode='dropout',bias_enu_m=[0,0,0])])
        c['evaluation']['output_dir']='evaluation'
        adapter=StateNode(c);loc=LocalizationNode(c);evaluation=EvaluationNode(c)
        for node in (adapter,loc,evaluation):node.set_parameters([Parameter('use_sim_time',value=True)])
        host=rclpy.create_node('ego_localization_test_driver');executor=SingleThreadedExecutor()
        for node in (host,adapter,loc,evaluation):executor.add_node(node)
        pubs={}
        for name,cls,topic,qos in [('clock',Clock,'/clock',SENSOR_QOS),('imu',Imu,'/drone1/imu/data',SENSOR_QOS),
            ('gnss',NavSatFix,'/drone1/gnss/fix',SENSOR_QOS),('datum',NavSatFix,'/simulation/geodetic_reference',LATCHED),
            ('truth',Odometry,'/drone1/ground_truth/odometry',SENSOR_QOS),('fiveg',FiveGObservations,'/drone1/five_ga/observations',SENSOR_QOS)]:pubs[name]=host.create_publisher(cls,topic,qos)
        received=[];states=[];statuses=[];metrics=[]
        host.create_subscription(Odometry,'/drone1/localization/odometry',lambda m:received.append(m),SENSOR_QOS)
        host.create_subscription(Odometry,'/drone1/state/odometry',lambda m:states.append(m),SENSOR_QOS)
        host.create_subscription(StateStatus,'/drone1/state/status',lambda m:statuses.append(m),10)
        host.create_subscription(String,'/drone1/localization/metrics',lambda m:metrics.append(json.loads(m.data)),10)
        def spin(seconds):
            deadline=time.monotonic()+seconds
            while time.monotonic()<deadline:executor.spin_once(timeout_sec=.001)
        start=1_800_000_000_000_000_000;origin=np.array([31.,-42.,12.])
        try:
            spin(.3)
            datum=NavSatFix(header=header(start,'map'),latitude=47.641468,longitude=-122.140165,altitude=122.)
            datum.status.status=0;pubs['datum'].publish(datum)
            for i in range(651):
                t=start+i*10_000_000;pubs['clock'].publish(Clock(clock=stamp(t)))
                # Stationary ideal input at a nonzero original-map position.
                m=Imu(header=header(t,'drone1/imu_link'));m.orientation.w=1.;m.linear_acceleration.z=9.80665;pubs['imu'].publish(m)
                if i%5==0:
                    truth=Odometry(header=header(t,'map'),child_frame_id='drone1/base_link');put(truth.pose.pose.position,origin);truth.pose.pose.orientation.w=1.;pubs['truth'].publish(truth)
                if i%10==0:
                    lat,lon,alt=inverse_geo(origin)
                    g=NavSatFix(header=header(t,'drone1/gnss_link'),latitude=lat,longitude=lon,altitude=alt);g.status.status=0
                    g.position_covariance=np.diag([.25,.25,.25]).ravel().tolist();g.position_covariance_type=2;pubs['gnss'].publish(g)
                    obs=FiveGObservations(header=header(t,'map'))
                    for j,a in enumerate(([0,0,0],[100,0,0],[0,100,0],[50,50,80])):
                        link=FiveGLink(gnb_id=str(j),measurement_valid=True,range_sigma_m=.15,prs_rsrp_dbm=-50.,measured_range_m=float(np.linalg.norm(origin-np.array(a))))
                        put(link.base_station_enu,a);obs.observations.append(link)
                    pubs['fiveg'].publish(obs)
                spin(.003)
            spin(.2)
            self.assertGreater(len(received),100);self.assertGreater(len(states),50)
            self.assertTrue(all(a.header.stamp.sec*10**9+a.header.stamp.nanosec<b.header.stamp.sec*10**9+b.header.stamp.nanosec for a,b in zip(received,received[1:])))
            self.assertEqual(states[-1].header.stamp,received[-1].header.stamp)
            self.assertEqual(states[-1].pose,received[-1].pose)
            self.assertLess(np.linalg.norm(loc.stream.filter.p),.01)
            self.assertGreater(loc.stream.stats['gnss_injected_dropout'],10)
            self.assertTrue(loc.stream.recoveries[0]['accepted']);self.assertGreater(loc.stream.stats['used_fiveg'],20)
            self.assertTrue(any(m.get('valid') for m in metrics))
            self.assertLess(metrics[-1]['summary']['rmse_3d_m'],.01)
            # New clock advances without estimator samples: adapter must time out.
            for j in range(30):
                pubs['clock'].publish(Clock(clock=stamp(start+int((7+j*.02)*1e9))));spin(.02)
            self.assertFalse(statuses[-1].valid)
            self.assertEqual(len(loc.get_subscriptions_info_by_topic('/drone1/ground_truth/odometry')),1) # adapter only
        finally:
            executor.shutdown()
            for node in (evaluation,loc,adapter,host):
                if node is adapter:adapter.lease.close()
                node.destroy_node()
            rclpy.shutdown();temporary.cleanup()


if __name__=='__main__':unittest.main()
