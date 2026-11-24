import unittest
from pathlib import Path
from unittest.mock import patch
import tempfile
import yaml
import rclpy
from evtol_localization.evaluation_node import EvaluationNode
from evtol_perception_interfaces.msg import ReferenceOrigin
from evtol_targeting.common import header


class EvaluationResilience(unittest.TestCase):
    def test_file_lock_keeps_real_ros_timer_and_publisher_alive(self):
        rclpy.init(domain_id=226)
        root=Path(__file__).resolve().parents[1]
        c=yaml.safe_load((root/'config/targeting.yaml').read_text(encoding='utf8'));c['_root']=root
        with tempfile.TemporaryDirectory() as folder:
            c['evaluation']['output_dir']=folder
            node=EvaluationNode(c)
            try:
                now=node.get_clock().now().nanoseconds
                node.reference(ReferenceOrigin(header=header(now),session_id='storage-test'))
                with patch.object(Path,'replace',side_effect=PermissionError('native file sharing error')):
                    node.tick()
                self.assertFalse(node.storage.status()['healthy'])
                # Invoke another real callback after the exception, then retry.
                node.last_saved=0;node.storage.next_retry=0;node.tick()
                self.assertTrue(node.storage.status()['healthy'])
                self.assertTrue((node.folder/'summary.json').is_file())
            finally:node.destroy_node();rclpy.shutdown()
