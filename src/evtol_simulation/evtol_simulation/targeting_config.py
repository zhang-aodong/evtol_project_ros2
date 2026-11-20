"""Keep the raw acquisition camera geometry consistent with the perception rig."""
def configure(simulation,targeting):
    s=targeting.get('surround',{})
    simulation['namespace']=targeting['namespace'];simulation['airsim'].update(targeting['airsim'])
    if not s.get('enabled',False):return simulation
    simulation['native_kinematics']=True;simulation['rates']['truth']=float(s.get('state_sample_hz',50.))
    camera=simulation['camera'];primary=s['cameras'][0]
    camera['reorder_window_s']=max(.35,camera.get('reorder_window_s',.2))
    camera.update(name=primary['name'],width=s['width'],height=s['height'],
        horizontal_fov_deg=s['horizontal_fov_deg'],xyz_frd_m=[0.,0.,0.],rpy_deg=primary['rpy_deg'],
        surround_cameras=[v['name'] for v in s['cameras']],surround_record_truth=bool(s.get('record_truth',False)))
    return simulation
