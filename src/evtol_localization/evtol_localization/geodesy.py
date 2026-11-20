"""WGS-84 ellipsoidal fix -> source-map ENU -> session ENU, once only."""
import numpy as np
from evtol_simulation.gnss_model import geodetic_to_ecef, enu_to_ecef_matrix


class GeoReference:
    def __init__(self,latitude,longitude,altitude,source_origin):
        if not np.isfinite([latitude,longitude,altitude]).all() or not -90<=latitude<=90 or not -180<=longitude<=180:raise ValueError('Invalid geodetic reference')
        self.ecef=geodetic_to_ecef(latitude,longitude,altitude)
        self.rotation=enu_to_ecef_matrix(latitude,longitude)
        self.origin=np.asarray(source_origin,float)

    def position(self,lat,lon,alt):
        if not np.isfinite([lat,lon,alt]).all() or not -90<=lat<=90 or not -180<=lon<=180:raise ValueError('Invalid GNSS fix coordinates')
        return self.rotation.T@(geodetic_to_ecef(lat,lon,alt)-self.ecef)-self.origin

    def anchor(self,source_map_position):return np.asarray(source_map_position,float)-self.origin
