import sys
import socket
import struct
import threading
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'/'evtol_simulation'))
from evtol_simulation.rpc import pack, unpack, Incomplete, AirSimRPC
from evtol_simulation.coordinates import NED_ENU, FRD_FLU, orientation, quaternion_matrix, extrinsic
from evtol_simulation.configuration import load
from evtol_simulation.preflight import inspect_settings
from evtol_simulation.gnss_model import GnssRawSimulator, geodetic_to_ecef
from evtol_simulation.radio_model import FiveGAState, FiveGASimulator
from evtol_simulation.backend import check_camera_projection
from evtol_simulation.runtime import RateGate
from evtol_simulation.semantic import SemanticDecoder

CONFIG=Path(__file__).resolve().parents[1]/'src'/'evtol_simulation'/'config'/'simulation.yaml'


class CoreTests(unittest.TestCase):
    def test_fast_semantic_decode_preserves_exact_colors(self):
        classes=load(CONFIG)['source']['semantic_segmentation']['classes']
        rng=np.random.default_rng(9)
        image=rng.integers(0,256,(64,64,3),dtype=np.uint8)
        for i,item in enumerate(classes): image[0,i]=item['color_rgb'][::-1]
        # Mixed channels from different valid colors must remain unknown.
        image[1,0]=[classes[1]['color_rgb'][2],classes[2]['color_rgb'][1],classes[3]['color_rgb'][0]]
        fast=SemanticDecoder(classes)(image)
        slow=SemanticDecoder(classes,prefer_opencv=False,prefer_native=False)(image)
        np.testing.assert_array_equal(fast,slow)
        self.assertEqual(fast[1,0],255)

    def test_native_semantic_decoder_handles_noncontiguous_views_exactly(self):
        classes=load(CONFIG)['source']['semantic_segmentation']['classes']
        decoder=SemanticDecoder(classes)
        self.assertIsNotNone(decoder.native, 'Build evtol_sensor_native before running the full suite')
        rng=np.random.default_rng(73)
        image=rng.integers(0,256,(128,256,3),dtype=np.uint8)
        for i,item in enumerate(classes):image[0,i]=item['color_rgb'][::-1]
        reference=SemanticDecoder(classes,prefer_opencv=False,prefer_native=False)
        for view in (image,image[::-1],image[:,::2]):
            np.testing.assert_array_equal(decoder(view),reference(view))
        with self.assertRaises(ValueError):decoder(image.astype(np.float32))

    def test_imu_retry_does_not_advance_publication_slot(self):
        gate=RateGate(100.)
        gate.accepted(0.)
        self.assertFalse(gate.ready(.009))
        self.assertTrue(gate.ready(.010))
        # A duplicate at 10 ms is not accepted; retry at 12.5 ms stays eligible.
        self.assertTrue(gate.ready(.0125))
        gate.accepted(.0125)
        self.assertAlmostEqual(gate.due,.020)
        # A long source stall skips expired slots rather than emitting a burst.
        gate.accepted(.054)
        self.assertAlmostEqual(gate.due,.060)

    def test_messagepack_roundtrip(self):
        for value in [None,True,False,0,127,128,-1,-33,-2**50,2**60,1.5,'你好',b'\x00\xff',
                      [1,False,{'timestamp':1234567890000000000}],{'x_val':3.,'image':b'abc'}]:
            wire=pack(value)
            self.assertEqual(unpack(wire),(value,len(wire)))
            for i in range(len(wire)):
                with self.assertRaises(Incomplete): unpack(wire[:i])

    def test_messagepack_standard_fixtures(self):
        # MessagePack specification fixmap/fixstr/fixarray, float32, all signed widths.
        fixtures=[('81a1789201ff',{'x':[1,-1]}),('ca3fc00000',1.5),('d080',-128),
                  ('d1ff00',-256),('d2ffff0000',-65536),('ceffffffff',4294967295),
                  ('c40300ff80',b'\x00\xff\x80')]
        for wire,expected in fixtures:
            self.assertEqual(unpack(bytes.fromhex(wire))[0],expected)

    def test_legacy_raw_binary_utf8_is_preserved(self):
        raw=unpack(bytes.fromhex('a2c2a3'))[0]
        self.assertEqual(raw.wire,bytes.fromhex('c2a3'))

    def test_fragmented_tcp_rpc(self):
        server=socket.socket()
        server.bind(('127.0.0.1',0)); server.listen()
        port=server.getsockname()[1]
        failures=[]
        def serve():
            try:
                connection,_=server.accept()
                with connection:
                    wire=bytearray()
                    while True:
                        wire.extend(connection.recv(1024))
                        try: request,_=unpack(wire); break
                        except Incomplete: continue
                    self.assertEqual(request[2],'getImuData')
                    response=pack([1,request[1],None,{'time_stamp':1234567891234567890}])
                    for b in response: connection.sendall(bytes([b]))
            except Exception as exc: failures.append(exc)
            finally: server.close()
        thread=threading.Thread(target=serve); thread.start()
        rpc=AirSimRPC(port=port)
        try: self.assertEqual(rpc.call('getImuData','Imu1','Drone1')['time_stamp'],1234567891234567890)
        finally: rpc.close()
        thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertFalse(failures)

    def test_ned_enu_and_orientation(self):
        np.testing.assert_allclose(NED_ENU@np.array([1,2,3]),[2,1,-3])
        q=orientation(dict(x_val=0.,y_val=0.,z_val=0.,w_val=1.))
        np.testing.assert_allclose(quaternion_matrix(q),NED_ENU@FRD_FLU,atol=1e-12)
        np.testing.assert_allclose(quaternion_matrix(q)@np.array([1,0,0]),[0,1,0],atol=1e-12)

    def test_camera_optical_axes(self):
        matrix=quaternion_matrix([-.5,.5,-.5,.5])
        np.testing.assert_allclose(matrix@np.array([0,0,1]),[1,0,0])
        np.testing.assert_allclose(matrix@np.array([1,0,0]),[0,-1,0])
        p,q=extrinsic([.3,0,0],[0,-10,0])
        np.testing.assert_allclose(p,[.3,0,0])
        self.assertAlmostEqual(np.linalg.det(quaternion_matrix(q)),1.)

    def test_capture_projection_uses_capture_fov_not_preview(self):
        c=load(CONFIG)['camera']
        matrix=[[0.,3.1715946,0.,0.],[0.,0.,-4.2287931,0.],
                [0.,0.,0.,10.],[-1.,0.,0.,0.]]
        info={'fov':89.9,'proj_mat':{'matrix':matrix}}
        check_camera_projection(info,c)
        matrix[0][1]=1.
        with self.assertRaises(ValueError): check_camera_projection(info,c)

    def test_source_rates_and_preflight(self):
        import json
        c=load(CONFIG)
        self.assertEqual(c['rates'],dict(camera=20.,lidar=20.,imu=100.,gnss=10.,truth=20.,radio=10.))
        settings=json.loads((CONFIG.parent/'airsim.20hz.json').read_text())
        self.assertEqual(inspect_settings(c,settings),[])
        settings['Vehicles']['Drone1']['Sensors']['Lidar1']['UpdateFrequency']=10
        self.assertTrue(any('UpdateFrequency' in x for x in inspect_settings(c,settings)))

    def test_gnss_model_repeatable(self):
        c=load(CONFIG)['source']['gnss_raw']
        origin=geodetic_to_ecef(47.,-122.,122.)
        a=GnssRawSimulator(47.,-122.,origin,c).simulate_epoch(origin,np.zeros(3))
        b=GnssRawSimulator(47.,-122.,origin,c).simulate_epoch(origin,np.zeros(3))
        self.assertEqual(a,b)
        self.assertEqual(len(a),10)
        self.assertTrue(all(o['pseudorange_m']>0 for o in a))

    def test_radio_observations_only(self):
        c=load(CONFIG)['source']['five_g_a']
        state=FiveGAState(0,12345,0.,'test',np.array([0.,0.,-10.]),np.zeros(3))
        a=FiveGASimulator(c,lambda *_:True)
        observations=a.simulate_epoch(state)
        self.assertEqual(len(observations),6)
        for obs in observations:
            if obs.measurement_valid:
                self.assertAlmostEqual(obs.measured_rtt_s,2*obs.measured_range_m/c['speed_of_light_mps'])
        self.assertFalse(hasattr(a,'solve_epoch'))


if __name__=='__main__': unittest.main()
