import struct
import unittest
from array import array
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, PointField
from evtol_visualization.preview import resize_image, thin_cloud
from evtol_visualization.schemas import expand_schema


class PreviewTests(unittest.TestCase):
    def test_semantic_nearest_and_padded_rows(self):
        msg = Image(width=4, height=2, encoding='rgb8', step=14,
                    data=bytes([255,0,0, 0,255,0, 0,0,255, 255,255,0, 99,99]*2))
        msg.header.stamp.sec, msg.header.frame_id = 42, 'camera'
        out, pixels = resize_image(msg, 2, semantic=True)
        self.assertEqual((out.width,out.height,out.step), (2,1,6))
        self.assertEqual(out.header, msg.header)
        self.assertEqual(pixels.tolist(), [[[255,0,0],[0,0,255]]])

    def test_organized_cloud_padding_fields_and_stamp(self):
        # 2 rows of 3 float x values with 4 bytes row padding.
        msg = PointCloud2(height=2, width=3, point_step=4, row_step=16,
            fields=[PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)],
            data=struct.pack('<8f', 0.,1.,2.,999.,3.,4.,5.,999.))
        msg.header.stamp.sec, msg.header.frame_id = 123, 'lidar'
        out = thin_cloud(msg, 3)
        self.assertEqual(out.header, msg.header)
        self.assertEqual(out.fields, msg.fields)
        self.assertEqual(out.width, 3)
        self.assertEqual(struct.unpack('<3f', out.data), (0.,2.,4.))
        self.assertEqual(out.row_step, 12)

    def test_schemas_expand_custom_and_standard_dependencies_once(self):
        for name in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/PointCloud2',
                     'evtol_sim_interfaces/msg/GnssObservations',
                     'evtol_sim_interfaces/msg/FiveGObservations', 'tf2_msgs/msg/TFMessage'):
            schema = expand_schema(name).decode()
            self.assertIn('MSG: std_msgs/Header', schema)
            self.assertEqual(schema.count('MSG: builtin_interfaces/Time\n'), 1)
            self.assertNotIn('MSG: std_msgs/msg/Header', schema)


if __name__ == '__main__':
    unittest.main()
