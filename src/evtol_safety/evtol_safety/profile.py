"""Waypoint mission for the user's horizontal/vertical speed-height profile."""
import numpy as np
from evtol_targeting.flight_profile import compile_profile,leg_speed
from .navigation import WaypointMission


class ProfileMission(WaypointMission):
    def __init__(self,settings):
        self.compiled=compile_profile(settings['profile'],settings.get('profile_speed_cap_mps',30.),
            settings.get('profile_acceleration_cap_mps2',5.))
        actual=dict(settings,waypoints_enu_m=self.compiled['waypoints_enu_m'])
        super().__init__(actual)
        self.index=1 if abs(self.points[0,2]-settings.get('landing_ground_z_m',0.))<.05 else 0
        self.current_speed_mps=0.

    def reference(self,position,velocity,now,flight_phase='flying'):
        if flight_phase=='takeoff' and self.index==1 and self.compiled['legs'][0]['kind']=='vertical' and self.points[1,2]>self.points[0,2]:
            if self.takeoff_target is None:
                self.takeoff_target=np.asarray(position).copy();self.takeoff_target[2]+=self.c.get('takeoff_height_m',1.5)
            self.phase='takeoff';return self.points[1].copy(),self.phase
        if flight_phase=='flying' and self.pass_through() and np.linalg.norm(self.points[self.index]-position)<=self.c['tolerance_m']:
            self.index+=1;self.settled_since=None
        return super().reference(position,velocity,now,flight_phase)

    def pass_through(self):
        if not 1<=self.index<len(self.points)-1:return False
        a=self.compiled['legs'][self.index-1];b=self.compiled['legs'][self.index]
        first=np.asarray(a['end_enu_m'])-a['start_enu_m'];second=np.asarray(b['end_enu_m'])-b['start_enu_m']
        return a['end_speed_mps']>0 and first@second/(a['length_m']*b['length_m'])>.999

    def speed_limit(self,position):
        if self.index==0:return self.c['speed_mps']
        leg=self.compiled['legs'][min(self.index-1,len(self.compiled['legs'])-1)]
        self.current_speed_mps=leg_speed(leg,position,self.c.get('profile_acceleration_cap_mps2',1.))
        return self.current_speed_mps


def make_mission(settings):
    return ProfileMission(settings) if settings.get('mode','waypoints')=='profile' else WaypointMission(settings)
