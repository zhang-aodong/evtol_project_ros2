"""Late Foxglove panels receive cached paths without republishing the mission."""
from pathlib import Path
from queue import SimpleQueue
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'.visualization_deps'))
from nav_msgs.msg import Path as NavPath
from rosgraph_msgs.msg import Clock
from rclpy.serialization import serialize_message
from evtol_targeting.common import header,stamp
from evtol_visualization.bridge import Bridge


class BridgePathContracts(unittest.TestCase):
    def bridge(self):
        topics={'/drone1/navigation/waypoint_path','/drone1/navigation/traveled_path'}
        return SimpleNamespace(path_topics=topics,path_data={},sim_ns=0,origin_data=None,
            channels={topic:Mock() for topic in topics},listener=SimpleNamespace(requests=SimpleQueue()),
            server=Mock(),get_logger=Mock(),publish_static=Mock())

    def test_path_is_replayed_when_panel_subscribes_later(self):
        bridge=self.bridge()
        for topic in bridge.path_topics:
            bridge.channels[topic].has_sinks.return_value=False
            payload=serialize_message(NavPath(header=header(123)))
            Bridge.forward(bridge,topic,payload)
            bridge.channels[topic].log.assert_not_called()
            bridge.listener.requests.put(topic)
        Bridge.replay_static(bridge)
        for topic in bridge.path_topics:
            bridge.channels[topic].log.assert_called_once()
            self.assertEqual(bridge.channels[topic].log.call_args.args[0],bridge.path_data[topic])

    def test_clock_reset_invalidates_path_replay_cache(self):
        bridge=self.bridge();bridge.sim_ns=200
        bridge.path_data={topic:b'old-session' for topic in bridge.path_topics}
        bridge.channels['/clock']=Mock()
        Bridge.forward(bridge,'/clock',serialize_message(Clock(clock=stamp(100))))
        self.assertEqual(bridge.path_data,{})
        bridge.server.clear_session.assert_called_once()
