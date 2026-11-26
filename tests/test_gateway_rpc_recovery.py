"""Control fault injection; no connection to UE or the running ROS domain."""
import copy
from pathlib import Path
import socket
import threading
import time
import unittest
from unittest.mock import Mock, patch

import rclpy
import yaml
from evtol_perception_interfaces.msg import ReferenceOrigin, StateStatus, VelocityCommand
from evtol_safety.gateway import Gateway
from evtol_safety.navigation import WaypointMission
from evtol_simulation.rpc import AirSimRPC
from evtol_targeting.common import header, stamp


ROOT=Path(__file__).resolve().parents[1]


class GatewayRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=231)

    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def node(self):
        c=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))
        c['_root']=ROOT;c['navigation_lidar']['enabled']=True
        c['control']['ttl_s']=1.;c['state']['timeout_s']=1.
        node=Gateway(c,execute=False);self.addCleanup(node.destroy_node)
        node.execute=True;node.phase='waiting'
        node.receive_origin(ReferenceOrigin(session_id='rpc-recovery-test'))
        self.refresh(node)
        return node

    def refresh(self,node):
        t=node.get_clock().now().nanoseconds
        node.status(StateStatus(session_id='rpc-recovery-test',valid=True))
        msg=VelocityCommand(header=header(t),session_id='rpc-recovery-test',
            sequence=node.sequence+1,reason='clear',valid_until=stamp(t+1_000_000_000))
        msg.velocity.x=.5
        node.receive(msg)

    @staticmethod
    def state(z):
        return dict(landed_state=1,kinematics_estimated=dict(
            position=dict(x_val=0.,y_val=0.,z_val=z),
            linear_velocity=dict(x_val=0.,y_val=0.,z_val=0.)))

    def test_takeoff_query_and_velocity_timeouts_recover_then_require_fresh_command(self):
        for failed_method in ('getMultirotorState','moveByVelocity'):
            with self.subTest(method=failed_method):
                node=self.node();calls=[];phases=[];owner=self
                transition=node.transition
                def phase(value):phases.append(value);transition(value)
                node.transition=phase
                class RPC:
                    def __init__(self,*args):self.last_method='';self.failed=False;self.queries=0;self.closed=0
                    def close(self):self.closed+=1
                    def call(self,method,*args):
                        self.last_method=method;calls.append((method,args,node.phase))
                        if method=='getMultirotorState':self.queries+=1
                        if not self.failed and method==failed_method and (method!='getMultirotorState' or self.queries==2):
                            self.failed=True;raise TimeoutError('injected socket timeout')
                        if method=='getMultirotorState':
                            if node.phase=='recovering':owner.refresh(node)
                            return owner.state(0. if self.queries==1 else -1.5)
                        if self.failed and method=='moveByVelocityZ' and node.phase!='recovering':
                            # No post-recovery command yet: gateway must hold,
                            # even though the pre-failure command is within TTL.
                            owner.refresh(node)
                        if self.failed and method=='moveByVelocity':node.stop.set()
                        return True
                with patch('evtol_safety.gateway.AirSimRPC',RPC):node.run()
                self.assertEqual(node.phase,'stopped');self.assertEqual(node.rpc_recoveries,1)
                self.assertIn(failed_method,node.rpc_last_error)
                recovery=phases.index('recovering')
                self.assertIn(phases[recovery+1],('takeoff','flying'))
                self.assertFalse(node.failsafe)
                motion=[(m,p) for m,a,p in calls if m in ('moveByVelocity','moveByVelocityZ')]
                held=motion.index(('moveByVelocityZ','recovering'))
                self.assertEqual(motion[held+1],('moveByVelocityZ','flying'))
                self.assertEqual(motion[held+2],('moveByVelocity','flying'))
                self.assertFalse(any(m=='armDisarm' and a[0] is False for m,a,p in calls))
                self.assertFalse(any(m=='enableApiControl' and a[0] is False for m,a,p in calls))
                self.assertEqual(sum(m=='armDisarm' for m,a,p in calls),1)
                self.assertFalse(any(m in ('land','takeoff') for m,a,p in calls))

    def test_repeated_hold_timeouts_preserve_altitude_and_can_be_stopped(self):
        node=self.node();node.phase='flying';node.watchdog_z=-12.;calls=[]
        class RPC:
            last_method='moveByVelocity'
            def close(self):pass
            def call(self,method,*args):
                self.last_method=method;calls.append((method,args))
                if method=='moveByVelocityZ':
                    if sum(m==method for m,a in calls)==3:node.stop.set()
                    raise socket.timeout('hold timeout')
                return True
        node.recover_rpc(RPC(),TimeoutError('initial timeout'))
        self.assertEqual(node.watchdog_z,-12.)
        self.assertEqual([a[2] for m,a in calls if m=='moveByVelocityZ'],[-12.]*3)
        self.assertFalse(any(m in ('armDisarm','land','takeoff','getMultirotorState') for m,a in calls))
        self.assertFalse(any(m=='enableApiControl' and not a[0] for m,a in calls))

    def test_stop_interrupts_unavailable_server_without_releasing_control(self):
        node=self.node();node.phase='takeoff';calls=[]
        class RPC:
            last_method='getMultirotorState'
            def close(self):pass
            def call(self,method,*args):
                self.last_method=method;calls.append((method,args))
                raise ConnectionRefusedError('UE unavailable')
        worker=threading.Thread(target=node.recover_rpc,args=(RPC(),TimeoutError('timeout')))
        worker.start();time.sleep(.15);node.stop.set();worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertTrue(calls)
        self.assertTrue(all(m=='enableApiControl' and a[0] for m,a in calls))

    def test_ambiguous_api_acquisition_reply_enters_hold_without_rearming(self):
        node=self.node();calls=[];owner=self
        class RPC:
            def __init__(self,*args):self.last_method='';self.failed=False
            def close(self):pass
            def call(self,method,*args):
                self.last_method=method;calls.append((method,args))
                if method=='enableApiControl' and not self.failed:
                    self.failed=True;raise TimeoutError('enable reply lost')
                if method=='getMultirotorState':return owner.state(-.7)
                if method=='moveByVelocityZ':node.stop.set()
                return True
        with patch('evtol_safety.gateway.AirSimRPC',RPC):node.run()
        self.assertTrue(node.failsafe)
        self.assertEqual(node.rpc_recoveries,1)
        self.assertFalse(any(m in ('armDisarm','takeoff','land','moveByVelocity') for m,a in calls))
        self.assertFalse(any(m=='enableApiControl' and a[0] is False for m,a in calls))

    def test_mission_does_not_advance_waypoint_during_recovery_or_failsafe(self):
        for phase in ('recovering','failsafe_hold'):
            c=copy.deepcopy(self.node().c['mission']);c['waypoints_enu_m']=[[0.,0.,3.],[0.,10.,3.]]
            mission=WaypointMission(c)
            for t in (1.,10.):
                target,status=mission.reference([0.,0.,3.],[0.,0.,0.],t,phase)
                self.assertEqual(mission.index,0);self.assertEqual(status,phase)


class RPCTransport(unittest.TestCase):
    def test_timeout_closes_socket_discards_partial_reply_and_tracks_method(self):
        broken=Mock();broken.recv.side_effect=socket.timeout('slow UE')
        rpc=AirSimRPC();rpc.buffer.extend(b'\x94')
        with patch('evtol_simulation.rpc.socket.create_connection',return_value=broken):
            with self.assertRaises(TimeoutError):rpc.call('getMultirotorState','Drone1')
        broken.close.assert_called_once()
        self.assertIsNone(rpc.sock);self.assertEqual(rpc.buffer,bytearray())
        self.assertEqual(rpc.last_method,'getMultirotorState')


if __name__=='__main__':unittest.main()
