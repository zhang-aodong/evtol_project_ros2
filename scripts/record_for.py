"""Finite-duration rosbag2 recording with an explicit, graceful writer close.

Uses the same native Recorder and QoS configuration as ros2 bag record. Useful
on Windows where an automated background process cannot receive terminal Ctrl+C.
"""
import argparse
from pathlib import Path
import time
import yaml
from ros2bag.api import convert_yaml_to_qos_profile
from rosbag2_py import Recorder, RecordOptions, StorageOptions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=10.)
    opts = parser.parse_args()
    if opts.duration <= 0:
        parser.error('duration must be positive')
    root = Path(__file__).resolve().parents[1]
    profile = root/'src'/'evtol_bringup'/'config'/'bag_qos.yaml'
    qos = yaml.safe_load(profile.read_text(encoding='utf8'))
    options = RecordOptions()
    options.topics = [*qos, '/tf', '/diagnostics']
    options.rmw_serialization_format = 'cdr'
    options.topic_qos_profile_overrides = convert_yaml_to_qos_profile(qos)
    options.disable_keyboard_controls = True
    recorder = Recorder(StorageOptions(uri=opts.output, storage_id='mcap'), options)
    try:
        recorder.start_spin()
        recorder.record()
        time.sleep(opts.duration)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        recorder.stop_spin()


if __name__ == '__main__':
    main()
