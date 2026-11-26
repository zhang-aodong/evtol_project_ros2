"""Console configuration, process ownership and speed-height mission contracts."""
import copy
import json
from pathlib import Path
import shutil
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from evtol_console.configuration import defaults,validate,artifacts,atomic
from evtol_console.engine import Engine
from evtol_console.processes import node_specs
from evtol_console.processes import OwnedProcess
import subprocess
import sys
from evtol_targeting.flight_profile import compile_profile,leg_speed
from evtol_safety.profile import ProfileMission

ROOT=Path(__file__).resolve().parents[1]


class ProfileContracts(unittest.TestCase):
    def profile(self):return dict(heading_deg=90.,points=[dict(speed_mps=v,height_m=h,distance_m=d) for v,h,d in [(0,0,100),(2,10,100),(4,10,30),(0,0,100)]])
    def test_only_vertical_when_height_changes_and_forward_when_level(self):
        p=compile_profile(self.profile(),5,1)
        np.testing.assert_allclose(p['waypoints_enu_m'],[[0,0,0],[0,0,10],[30,0,10],[30,0,0]],atol=1e-10)
        self.assertEqual([l['kind'] for l in p['legs']],['vertical','horizontal','vertical'])
        self.assertAlmostEqual(p['legs'][0]['acceleration_mps2'],.2)
        self.assertAlmostEqual(p['legs'][1]['nominal_duration_s'],10.)
    def test_speed_depends_on_measured_progress_not_wall_clock(self):
        leg=compile_profile(self.profile(),5,1)['legs'][1]
        self.assertAlmostEqual(leg_speed(leg,[15,0,10],1),np.sqrt(10.02),places=4)
        self.assertAlmostEqual(leg_speed(leg,[15,30,10],1),leg_speed(leg,[15,0,10],1))
        self.assertGreater(leg_speed(compile_profile(self.profile(),5,1)['legs'][0],[0,0,0],1),0)
    def test_invalid_speeds_heights_zero_motion_and_acceleration_rejected(self):
        for change in [('speed_mps',float('nan')),('speed_mps',-1),('height_m',-1),('distance_m',0)]:
            p=self.profile();p['points'][1][change[0]]=change[1]
            with self.assertRaises(ValueError):compile_profile(p,5,1)
        p=self.profile();p['points'][1]['speed_mps']=0
        with self.assertRaises(ValueError):compile_profile(p,5,1)
        p=self.profile();p['points'][1]['height_m']=.1
        with self.assertRaises(ValueError):compile_profile(p,5,1)
    def test_profile_starts_first_vertical_leg_during_safety_takeoff(self):
        settings=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))['mission']
        settings.update(profile=self.profile(),profile_speed_cap_mps=5,profile_acceleration_cap_mps2=1)
        m=ProfileMission(settings);target,phase=m.reference([0,0,0],[0,0,0],0,'takeoff')
        np.testing.assert_allclose(target,[0,0,10]);self.assertEqual(phase,'takeoff');self.assertEqual(m.index,1)
        self.assertFalse(m.pass_through())
    def test_collinear_legs_pass_through_but_turns_settle(self):
        p=self.profile();p['points'].insert(3,dict(speed_mps=3,height_m=10,distance_m=30))
        s=yaml.safe_load((ROOT/'config/targeting.yaml').read_text(encoding='utf8'))['mission']
        s.update(profile=p,profile_speed_cap_mps=5,profile_acceleration_cap_mps2=1)
        m=ProfileMission(s);m.index=2;self.assertTrue(m.pass_through())
        m.reference([30,0,10],[3,0,0],1,'flying');self.assertEqual(m.index,3);self.assertFalse(m.pass_through())


class ConfigurationContracts(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        for name in ['config/targeting.yaml','config/actors.yaml','config/machine.local.json','src/evtol_simulation/config/airsim.navigation_2k.json']:
            target=self.root/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,target)
        self.d=defaults(self.root)
    def test_fused_delay_is_included_in_scan_budget_and_braking(self):
        from evtol_safety.velocity import LocalVelocityAvoidance
        self.d['localization']['state_source']='fused'
        _,c,_,_,_=artifacts(self.d,self.root)
        self.assertGreaterEqual(c['safety']['cloud_timeout_s'],c['localization']['reorder_s']+.1)
        safe=LocalVelocityAvoidance(c['safety']['local_velocity'],c['safety']['cloud_timeout_s'],c['control']['duration_s'],0.)
        shorter=LocalVelocityAvoidance(c['safety']['local_velocity'],.2,c['control']['duration_s'],0.)
        self.assertLess(safe.speed_limit,shorter.speed_limit)
    def test_count_controls_real_ue_vehicles_and_preserves_ego_sensor_settings(self):
        self.d['actors']['count']=1;self.d['actors']['agents']=self.d['actors']['agents'][:1]
        d,c,scenario,ue,p=artifacts(self.d,self.root)
        self.assertEqual(set(ue['Vehicles']),{'Drone1','Drone2'})
        self.assertEqual(len(scenario['interference_drones']['agents']),1)
        self.assertEqual(scenario['interference_drones']['agents'][0]['coordinate_mode'],'initial_enu')
        original=json.loads((self.root/self.d['ue']['settings']).read_text())
        generated=ue['Vehicles']['Drone1']
        self.assertEqual(set(generated['Cameras']),{'front_left','front_center','front_right'})
        self.assertEqual(generated,original['Vehicles']['Drone1'])
        self.assertEqual(c['mission']['waypoints_enu_m'],self.d['mission']['waypoints_enu_m'])

    def test_force_interrupts_cleanup_and_ends_only_owned_processes(self):
        entered=threading.Event()
        def cleanup(service,timeout,cancel=None):
            entered.set()
            if cancel.wait(10):raise RuntimeError('interrupted')
            raise TimeoutError('not interrupted')
        engine=Engine(self.root,ros_stop=cleanup);self.addCleanup(engine.close)
        external=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])
        self.addCleanup(lambda:external.poll() is None and external.kill())
        children=[]
        for name in ('actors','ue'):
            p=OwnedProcess(name,[sys.executable,'-c','import time;time.sleep(60)'],self.root,self.root/(name+'.log'))
            children.append(p);engine.processes[name]=p
        self.assertTrue(engine.request({'action':'stop_all'})['ok'])
        self.assertTrue(entered.wait(2),'Cleanup did not start')
        started=time.monotonic();result=engine.request({'action':'force_stop_all'})
        self.assertTrue(result['ok'],result)
        self.assertLess(time.monotonic()-started,.5,'Force request blocked behind normal cleanup')
        end=time.monotonic()+8
        while (engine.busy or engine.force_running) and time.monotonic()<end:time.sleep(.02)
        self.assertFalse(engine.busy);self.assertFalse(engine.force_running)
        self.assertTrue(all(p.process.poll() is not None for p in children))
        self.assertIsNone(external.poll(),'External process must not be terminated')
        self.assertEqual(next(o for o in engine.operations if o['id']==result['data']['operation_id'])['state'],'done')

    def test_force_prevents_a_late_startup_launch(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.force_event.set()
        with patch.object(engine,'factory') as factory:
            with self.assertRaises(RuntimeError):engine.launch('late',[],self.root)
            factory.assert_not_called()

    def test_child_logs_and_structured_events_are_persisted(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        from types import SimpleNamespace
        path=self.root/'child.log';path.write_bytes(b'[WARN] waiting for land\npartial')
        engine.processes['actors']=SimpleNamespace(log=path,state=lambda:dict(running=False),process=SimpleNamespace(pid=0,poll=lambda:0),stop=lambda:None)
        engine.read_process_logs()
        with path.open('ab') as f:f.write(b' line\n')
        engine.read_process_logs()
        events=[json.loads(line) for line in engine.log_file.read_text(encoding='utf8').splitlines()]
        self.assertTrue(any(e['source']=='actors' and e['level']=='WARN' for e in events))
        self.assertTrue(any(e['message']=='partial line' for e in events))
    def test_profile_compiles_into_normal_mission_waypoints(self):
        self.d['mission']['mode']='profile';d,c,sc,ue,p=artifacts(self.d,self.root)
        self.assertEqual(c['mission']['waypoints_enu_m'],[[0,0,0],[0,0,10],[0,30,10],[0,30,0]])
        self.assertEqual(c['mission']['mode'],'profile')
    def test_bad_actor_count_ego_name_and_envelope_fail(self):
        for mutate in [lambda d:d['actors'].update(count=99),lambda d:d['actors']['agents'][0].update(name='Drone1'),
                lambda d:d['mission'].update(speed_mps=30,max_speed_mps=30)]:
            d=copy.deepcopy(self.d);mutate(d)
            with self.assertRaises(ValueError):artifacts(d,self.root)
    def test_save_is_atomic_and_does_not_overwrite_base_yaml(self):
        old=(self.root/'config/targeting.yaml').read_bytes()
        engine=Engine(self.root);self.addCleanup(engine.close)
        result=engine.request({'action':'save','payload':self.d});self.assertTrue(result['ok'],result)
        self.assertEqual(json.loads((self.root/'config/console.local.json').read_text()),self.d)
        self.assertEqual((self.root/'config/targeting.yaml').read_bytes(),old)
    def test_only_allowlisted_operations_and_busy_requests_rejected(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        self.assertFalse(engine.request({'action':'powershell','payload':{'cmd':'bad'}})['ok'])
        engine.busy=True
        self.assertFalse(engine.request({'action':'start_ue'})['ok']);engine.busy=False
        self.assertTrue(engine.request({'action':'preview','payload':self.d})['ok'])
    def test_startup_plan_separates_ue_and_ros_and_shadow_has_no_execute(self):
        c=yaml.safe_load((self.root/'config/targeting.yaml').read_text())
        specs=node_specs(c,'targeting.yaml','sim.yaml',dict(execute=False,actors=False,evaluate=False))
        self.assertEqual(len([1 for n,m,a in specs if m=='evtol_simulation.sensor_node']),6)
        self.assertFalse(any('--execute' in a for n,m,a in specs))
        self.assertFalse(any(n=='actors' for n,m,a in specs))
        self.assertFalse(any('UE4Editor' in m for n,m,a in specs))
    def test_ue_start_failure_does_not_claim_running_or_touch_external_processes(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.config['ue']['editor']='Z:/missing/editor.exe'
        response=engine.request({'action':'start_ue'});self.assertTrue(response['ok'])
        end=time.monotonic()+3
        while engine.busy and time.monotonic()<end:time.sleep(.02)
        self.assertEqual(engine.operations[-1]['state'],'failed');self.assertFalse(engine.processes)
    def test_ros_start_without_ue_reports_failure_before_any_child(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        with patch.object(engine,'rpc') as rpc:
            rpc.return_value.call.side_effect=ConnectionError('No UE')
            with self.assertRaises(ConnectionError):engine.start_ros()
        self.assertFalse(engine.processes)

    def test_inflight_algorithm_failure_retains_gateway_instead_of_cleanup(self):
        from types import SimpleNamespace
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.config['run']['execute']=True
        engine.telemetry['flight']=dict(phase='flying')
        process=lambda code:SimpleNamespace(process=SimpleNamespace(poll=lambda:code))
        engine.processes=dict(safety=process(1),gateway=process(None),truth_sensor=process(None))
        try:
            with patch.object(engine,'rpc') as rpc,patch.object(engine,'stop_ros') as cleanup:
                engine.ros_stop=lambda service,timeout:'holding'
                rpc.return_value.call.return_value=True
                engine.last_health=0.;engine.monitor()
                cleanup.assert_not_called()
                self.assertTrue(engine.telemetry['runtime_fault']['hold_requested'])
                self.assertEqual(engine.telemetry['runtime_fault']['failed'],['safety'])
        finally:engine.processes={}

    def test_rpc_outage_and_critical_node_exit_keep_recovering_gateway_alive(self):
        from types import SimpleNamespace
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.config['run']['execute']=True
        engine.telemetry['flight']=dict(phase='recovering')
        process=lambda code:SimpleNamespace(process=SimpleNamespace(poll=lambda:code))
        engine.processes=dict(safety=process(1),gateway=process(None),truth_sensor=process(None))
        try:
            with patch.object(engine,'rpc') as rpc,patch.object(engine,'stop_ros') as cleanup:
                engine.ros_stop=lambda service,timeout:'holding'
                rpc.return_value.call.side_effect=TimeoutError('UE RPC unavailable')
                engine.last_health=0.;engine.monitor()
                cleanup.assert_not_called()
                self.assertTrue(engine.telemetry['runtime_fault']['hold_requested'])
        finally:engine.processes={}

    def test_evaluation_or_visualization_exit_does_not_hold_or_stop_flight(self):
        from types import SimpleNamespace
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.config['run']['execute']=True;engine.telemetry['flight']=dict(phase='flying')
        process=lambda code:SimpleNamespace(process=SimpleNamespace(poll=lambda:code))
        engine.processes=dict(ego_localization_evaluation=process(1),ue_boxes=process(1),gateway=process(None),safety=process(None))
        try:
            with patch.object(engine,'rpc') as rpc,patch.object(engine,'stop_ros') as cleanup,patch.object(engine,'ros_stop') as hold:
                rpc.return_value.call.return_value=True
                engine.last_health=0.;engine.monitor();engine.last_health=0.;engine.monitor()
                cleanup.assert_not_called();hold.assert_not_called()
                self.assertNotIn('runtime_fault',engine.telemetry)
                self.assertEqual(engine.telemetry['runtime_warnings']['failed'],['ego_localization_evaluation','ue_boxes'])
                self.assertEqual(sum('Noncritical nodes exited' in e['message'] for e in engine.logs),1)
        finally:engine.processes={}

    def test_node_criticality_follows_run_state_source_and_unknown_nodes_fail_closed(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.runtime_config=dict(state=dict(source='truth'),navigation_lidar=dict(enabled=True),localization=dict(mode='gnss_imu'))
        self.assertFalse(engine.flight_critical('ego_localization'))
        self.assertFalse(engine.flight_critical('lidar_sensor'))
        self.assertFalse(engine.flight_critical('gnss_sensor'))
        self.assertTrue(engine.flight_critical('navigation_lidar'))
        self.assertTrue(engine.flight_critical('unknown_new_algorithm'))
        engine.runtime_config['state']['source']='fused'
        self.assertTrue(engine.flight_critical('ego_localization'))
        self.assertTrue(engine.flight_critical('imu_sensor'))
        self.assertFalse(engine.flight_critical('radio_sensor'))
        engine.runtime_config['localization']['mode']='gnss_imu_5g'
        self.assertTrue(engine.flight_critical('radio_sensor'))
    def test_windows_ros_diagnostic_byte_level_is_json_serializable(self):
        import sys
        from types import SimpleNamespace
        sys.path.insert(0,str(ROOT/'.visualization_deps'))
        from evtol_console.server import ConsoleBridge
        from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
        engine=Engine(self.root);self.addCleanup(engine.close)
        msg=DiagnosticArray(status=[DiagnosticStatus(name='sensor',level=DiagnosticStatus.OK,message='streaming')])
        ConsoleBridge.diagnostics(SimpleNamespace(engine=engine),msg)
        value=json.loads(json.dumps(engine.status()))
        self.assertEqual(value['telemetry']['diagnostics']['sensor']['level'],0)
    def test_executing_gateway_waits_for_real_fresh_successful_inference(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.telemetry['diagnostics']={'/drone1/vision':dict(level=0,received=time.time(),values={'samples':'0'})}
        self.assertFalse(engine.vision_ready('drone1'))
        info=engine.telemetry['diagnostics']['/drone1/vision'];info['values']['samples']='1'
        self.assertTrue(engine.vision_ready('drone1'))
        info['received']=time.time()-4.
        self.assertFalse(engine.vision_ready('drone1'))
        info.update(received=time.time(),level=1)
        self.assertFalse(engine.vision_ready('drone1'))

    def test_navigation_arming_gate_rejects_partial_stale_or_failed_data(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        self.assertFalse(engine.navigation_ready())
        engine.telemetry.update(mission=dict(phase='waiting',reason='mission_waiting',navigation_ready_s=.6),mission_received_wall=time.time())
        self.assertTrue(engine.navigation_ready())
        engine.telemetry['mission']['navigation_ready_s']=.1;self.assertFalse(engine.navigation_ready())
        engine.telemetry['mission'].update(navigation_ready_s=2.,reason='LiDAR stale');self.assertFalse(engine.navigation_ready())
        engine.telemetry['mission']['reason']='mission_waiting';engine.telemetry['mission_received_wall']=time.time()-1.
        self.assertFalse(engine.navigation_ready())

    def test_existing_configuration_adds_explicit_actor_initialization_without_changing_routes(self):
        from evtol_console.experiment_config import upgrade
        original=copy.deepcopy(self.d);original['actors'].pop('initialization',None)
        upgraded=upgrade(original)
        self.assertEqual(upgraded['actors']['initialization'],'first_waypoint')
        self.assertEqual(upgraded['actors']['agents'],original['actors']['agents'])
        _,_,scenario,_,_=artifacts(upgraded,self.root)
        self.assertEqual(scenario['interference_drones']['initialization'],'first_waypoint')
        upgraded['actors']['initialization']='ground';upgraded['run']['execute']=True
        _,_,scenario,_,_=artifacts(upgraded,self.root)
        self.assertEqual(scenario['interference_drones']['initialization'],'ground')
        self.assertTrue(scenario['interference_drones']['wait_for_ego_flying'])

    def test_obstacle_startup_failure_keeps_vehicle_and_collision_reason(self):
        engine=Engine(self.root);self.addCleanup(engine.close)
        engine.telemetry.update(actors={'actors':{'Drone15':dict(phase='failed',reason='collision=airpiane3')}},actors_received_wall=time.time())
        with self.assertRaisesRegex(RuntimeError,'Drone15: collision=airpiane3'):
            engine.wait_for_actors({'model':{}},[{'name':'Drone15'}])

    def test_origin_waits_until_ego_has_stopped_falling_without_arming(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        clock=[0.];rpc=Mock()
        def call(method,*args):
            self.assertEqual(method,'getMultirotorState')
            falling=clock[0]<1.
            return dict(kinematics_estimated=dict(position=dict(x_val=0.,y_val=0.,z_val=min(clock[0],1.)),
                linear_velocity=dict(x_val=0.,y_val=0.,z_val=1. if falling else 0.)))
        rpc.call.side_effect=call
        def wait(seconds):clock[0]+=seconds
        engine=SimpleNamespace(log=lambda message:None,rpc=lambda timeout:rpc,check_abort=lambda:None,
            closed=SimpleNamespace(wait=wait))
        with patch('evtol_console.engine.time.monotonic',side_effect=lambda:clock[0]):
            Engine.wait_for_ego_settled(engine,{'airsim':{'vehicle':'Drone1'}})
        self.assertGreaterEqual(clock[0],1.8)
        rpc.close.assert_called_once()

    def test_origin_rejects_an_ego_that_keeps_falling(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        clock=[0.];rpc=Mock()
        def call(method,*args):
            if method=='simGetCollisionInfo':return dict(object_name='blocked_spawn')
            return dict(kinematics_estimated=dict(position=dict(x_val=0.,y_val=0.,z_val=clock[0]),
                linear_velocity=dict(x_val=0.,y_val=0.,z_val=8.)))
        rpc.call.side_effect=call
        def wait(seconds):clock[0]+=seconds
        engine=SimpleNamespace(log=lambda message:None,rpc=lambda timeout:rpc,check_abort=lambda:None,
            closed=SimpleNamespace(wait=wait))
        with patch('evtol_console.engine.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(RuntimeError,'blocked_spawn'):
                Engine.wait_for_ego_settled(engine,{'airsim':{'vehicle':'Drone1'}})
        rpc.close.assert_called_once()
