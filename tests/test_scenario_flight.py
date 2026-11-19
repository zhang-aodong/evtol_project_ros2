"""Regress moving spawns, short AirSim paths, and experiment start barriers."""
import copy
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
import numpy as np

from evtol_scenario.flight import PositionFlight, vector, xyz


class Stop:
    def __init__(self, rpc):self.rpc=rpc;self.stopped=False
    def is_set(self):return self.stopped
    def wait(self, seconds):
        # UE keeps integrating motion between feedback RPCs.
        if not self.rpc.frozen:self.rpc.position+=self.rpc.velocity*seconds
        self.rpc.now+=seconds
        return self.stopped


class RPC:
    timeout=40.
    def __init__(self, position=(0.,0.,0.), velocity=(0.,0.,0.), frozen=False):
        self.position=np.array(position,dtype=float);self.velocity=np.array(velocity,dtype=float)
        self.now=0.;self.calls=[];self.frozen=frozen
    def call(self, method, *args):
        self.calls.append((method,copy.deepcopy(args)))
        if method=='getMultirotorState':
            return dict(kinematics_estimated=dict(position=xyz(self.position),linear_velocity=xyz(self.velocity)))
        if method=='simSetKinematics':
            self.position=vector(args[0]['position']);self.velocity=vector(args[0]['linear_velocity'])
        elif method=='moveByVelocity':
            self.now+=args[3]
            if not self.frozen:
                self.velocity+=.6*(np.asarray(args[:3])-self.velocity)
                self.position+=self.velocity*args[3]
        elif method=='simGetCollisionInfo':return dict(object_name='blocked_mesh')
        elif method=='moveToPosition':
            self.now+=1.
            if not self.frozen:self.position=np.asarray(args[:3])+[.6,0.,0.]
            return True  # AirSim can claim success while measured error remains.
        return True


class FlightContracts(unittest.TestCase):
    def controller(self, **kwargs):
        rpc=RPC(**kwargs);stop=Stop(rpc)
        return rpc,stop,PositionFlight(rpc,'Drone15',stop,clock=lambda:rpc.now)

    def test_short_path_is_actually_flown_instead_of_repeated_noops(self):
        rpc,_,flight=self.controller()
        actual,error=flight.move([.65,0.,0.],2.,10.)
        self.assertLessEqual(error,.5)
        self.assertGreater(actual[0],.15)
        self.assertTrue(any(m=='moveByVelocity' for m,_ in rpc.calls))

    def test_long_leg_uses_configured_speed_without_blocking_path_api(self):
        rpc,_,flight=self.controller()
        _,error=flight.move([10.,0.,-7.],2.,10.)
        self.assertLessEqual(error,.5)
        self.assertFalse(any(m=='moveToPosition' for m,_ in rpc.calls))
        self.assertTrue(any(m=='moveByVelocity' for m,_ in rpc.calls))
        commands=[np.linalg.norm(a[:3]) for m,a in rpc.calls if m=='moveByVelocity']
        self.assertGreater(max(commands),1.5)
        self.assertLessEqual(max(commands),2.+1e-8)

    def test_airborne_spawn_clears_falling_state_before_arming(self):
        rpc,_,flight=self.controller(position=(0.,0.,100.),velocity=(0.,0.,8.))
        self.assertTrue(flight.place([0.,0.,-8.]))
        methods=[m for m,_ in rpc.calls]
        self.assertLess(methods.index('simSetKinematics'),methods.index('armDisarm'))
        self.assertNotIn('takeoff',methods)
        self.assertLess(np.linalg.norm(rpc.position-[0.,0.,-8.]),.5)

    def test_collision_cannot_be_reported_as_success(self):
        _,_,flight=self.controller(frozen=True)
        with self.assertRaisesRegex(RuntimeError,'blocked_mesh'):flight.move([1.,0.,0.],1.,1.)

    def test_cancelled_controller_does_not_send_movement(self):
        rpc,stop,flight=self.controller();stop.stopped=True
        self.assertIsNone(flight.move([1.,0.,0.],1.,10.))
        self.assertFalse(any(m.startswith('move') for m,_ in rpc.calls))

    def test_invalid_state_and_waypoints_rejected(self):
        _,_,flight=self.controller()
        with self.assertRaises(ValueError):flight.move([math.nan,0.,0.],1.,10.)
        _,_,flight=self.controller(position=(math.nan,0.,0.))
        with self.assertRaisesRegex(RuntimeError,'non-finite'):flight.move([1.,0.,0.],1.,10.)

    def test_only_current_session_ego_flying_releases_staged_actors(self):
        from std_msgs.msg import String
        from evtol_scenario.actors import Actors
        node=SimpleNamespace(origin=SimpleNamespace(session_id='current'),motion=threading.Event(),
                             get_logger=lambda:SimpleNamespace(info=lambda message:None))
        for session,phase in [('old','flying'),('current','arming'),('current','takeoff')]:
            Actors.receive_flight(node,String(data=json.dumps(dict(session_id=session,phase=phase))))
            self.assertFalse(node.motion.is_set())
        Actors.receive_flight(node,String(data=json.dumps(dict(session_id='current',phase='flying'))))
        self.assertTrue(node.motion.is_set())


if __name__=='__main__':unittest.main()
