# UE 4.27 sensor extension snapshot

This directory contains the sensor extension source files from the currently
running UE 4.27/CosysAirSim project. It is a reproducibility snapshot, not a
standalone AirSim distribution or a generic patch for every AirSim version.
Original copyright/license notices are retained in the source files.

Copy the complete matching UE project, assets and AirSim plugin when migrating.
These files include the asynchronous paired camera push/readback ring, XYZ and
object-identity LiDAR rings, capture visibility support, and configurable LiDAR
update scheduling. They preserve the layouts consumed by the Python readers.

Run `scripts/ue_sensor_support.ps1 -PluginDirectory <AirSim plugin path>` to verify
SHA-256 checksums. Use `-Apply` only when deliberately restoring these versions
over the matching plugin base; changed files are backed up first. Rebuild UE
after applying. Do not copy only compiled DLLs between different UE releases.

The 2026-11-10 frequency repair includes fixed-step scheduling, equivalent
LiDAR ray calculation and per-scan metadata caching, and an Unreal IMU adapter
that retains original ImuSimple outputs in a versioned Windows history ring.
The Python adapter never replaces empty point clouds with old frames or
interpolates/repeats IMU samples. See docs/传感器频率修复验证20261110.md for
the measured limits, 100 ms IMU buffer delay, build steps and acceptance reports.
