# eVTOL Offline Fusion Core

This directory contains the C++ offline fusion program for AirSim eVTOL data.
The code reads raw text/image files directly and does not require ROS or rosbag.

## Supported Dataset Layouts

The loader auto-detects two layouts:

- Legacy flat text data such as `simdata_v2`.
- Structured experiment data containing `ground_truth/truth.csv` and
  `sensors/{imu,gnss,camera,lidar,5g_a}`.

For the current structured experiment, use the Windows drive through WSL without
duplicating the approximately 2 GB dataset:

```bash
cd /home/fizz/eVTOL/offline_fusion
./build/linux_visual/offline_fusion \
  --data /mnt/e/data/Airsim/exp_20260729_222105_353335 \
  --load-only
```

The structured loader:

- preserves the absolute nanosecond origin in `Dataset::time_origin_ns`;
- converts timestamps to relative seconds only after integer subtraction;
- loads IMU, GNSS raw/fix, camera metadata, LiDAR metadata, and 5G-A raw ranges;
- validates every referenced image and LiDAR BIN size;
- reads LiDAR points as little-endian `float32 [x, y, z, intensity]` on demand.

The structured experiment truth file has position and velocity but no attitude.
The Ceres path therefore initializes roll/pitch from gravity-consistent IMU samples,
initializes heading from the first stable SPP-like horizontal-velocity segment, and
propagates attitude with gyroscope measurements. The first truth position is used once
as the global NED translation anchor for fixed virtual satellites. Every later GNSS
epoch is solved independently from pseudorange, azimuth, elevation, and SNR; no
pseudorange differencing is used. `gps_fix` is loaded for interface and baseline
evaluation only and is never added to the estimator.

Run the structured GNSS SPP-like + IMU solution:

```bash
cd /home/fizz/eVTOL/offline_fusion
./build/linux_visual/offline_fusion \
  --data /mnt/e/data/Airsim/exp_20260729_222105_353335 \
  --disable-visual \
  --output results/exp_20260729_222105_353335/gnss_spp_imu/trajectory_est.csv

python3 tools/evaluate_trajectory.py \
  --estimate results/exp_20260729_222105_353335/gnss_spp_imu/trajectory_est.csv \
  --data /mnt/e/data/Airsim/exp_20260729_222105_353335 \
  --output results/exp_20260729_222105_353335/gnss_spp_imu/evaluation
```

Historical relative-only snapshot (`--disable-gnss-fix`):

```text
GNSS raw only:
  states: 134
  RMSE 3D: 3.918092 m
  max error: 8.084760 m

GNSS raw + IMU:
  states: 134
  Ceres termination: CONVERGENCE
  initial/final cost: 2646.652 / 31.562
  RMSE 3D: 3.293243 m
  max error: 4.615709 m
  groundtruth used for initialization: false

GNSS raw + IMU + Vision:
  states: 503
  camera frames processed: 373
  visual keyframes: 372
  landmarks: 200
  reprojection factors: 1127
  initial visual reprojection RMSE: 6.218 px
  Ceres termination: NO_CONVERGENCE (60-iteration limit, solution usable)
  initial/final cost: 5929.115 / 61.774
  RMSE 3D: 3.323799 m
  max error: 4.897062 m
  groundtruth used for initialization: false

GNSS raw + IMU + LiDAR:
  states: 785
  LiDAR frames: 1324
  candidate/accepted relative-pose pairs: 657 / 215
  mean accepted ICP RMSE: 0.483735 m
  mean accepted overlap: 0.977973
  Ceres termination: CONVERGENCE
  initial/final cost: 737.732 / 37.234
  RMSE 3D: 3.336867 m
  max error: 4.798453 m
  groundtruth used for initialization: false
```

The remaining approximately 2.8 m Down-axis bias is already present in the
`gnss_raw`-only solution; IMU fusion reduces the Down RMSE from 3.412 m to 2.868 m.
The first structured-data visual run is stable and uses real reprojection residuals,
but its 3D RMSE is 0.031 m worse than the GNSS+IMU baseline. It therefore verifies
integration, not an accuracy improvement. The next visual work should focus on
recording/calibrating camera intrinsics, improving landmark selection, and adding a
sliding-window or better-conditioned landmark parameterization.

The first LiDAR run uses body-aligned sensor-local point clouds and point-to-plane
ICP between 0.2 s keyframes. Target normals are estimated from local nearest
neighbors, and accepted relative poses are added to Ceres with robust loss. A
0.4 m maximum ICP translation correction rejects geometrically ambiguous pairs.
The final 3D RMSE is 0.044 m worse than the GNSS+IMU baseline, so this run verifies
stable factor integration rather than an absolute-accuracy improvement. Most of the
remaining error is a global Down-axis bias that relative LiDAR factors cannot observe.

README-aligned absolute-factor snapshot:

```text
GNSS raw + IMU + per-epoch GPS fix:
  states: 134
  raw-GNSS scalar factors: 402
  absolute GPS-fix scalar factors: 402
  Ceres termination: CONVERGENCE
  RMSE 3D: 0.450325 m
  max error: 0.675836 m

GNSS raw + IMU + GPS fix + 5G-A:
  states: 1438
  accepted 5G-A range factors: 7196
  low-RSRP observations rejected: 694
  Ceres termination: CONVERGENCE
  RMSE 3D: 0.167856 m
  max error: 0.648090 m

GNSS raw + IMU + GPS fix + 5G-A + Vision + LiDAR:
  states: 1755
  visual reprojection factors: 1100
  LiDAR relative-pose factors: 215
  5G-A range factors: 7196
  Ceres termination: NO_CONVERGENCE (60-iteration limit, solution usable)
  initial/final cost: 18805.738 / 458.588
  RMSE 3D: 0.163611 m
  max error: 0.681747 m
```

The 5G-A implementation uses only receiver-observable fields:
`measurement_valid`, `measured_range_m`, `range_sigma_m`, and `prs_rsrp_dbm`.
It rejects PRS-RSRP below -32 dBm and applies a Cauchy loss to the remaining
Multi-RTT residuals. The simulator-only `true_range_m`, `is_los`, and
`nlos_bias_m` fields are never used by the estimator.

Run the complete structured visual configuration:

```bash
cd /home/fizz/eVTOL/offline_fusion
./build/linux_visual/offline_fusion \
  --data /mnt/e/data/Airsim/exp_20260729_222105_353335 \
  --camera-fx 3248 --camera-fy 3248 \
  --visual-max-landmarks 300 \
  --output results/exp_20260729_222105_353335/gnss_imu_vision/trajectory_est.csv
```

Run the structured LiDAR configuration without visual factors:

```bash
cd /home/fizz/eVTOL/offline_fusion
./build/linux_visual/offline_fusion \
  --data /mnt/e/data/Airsim/exp_20260729_222105_353335 \
  --disable-visual \
  --enable-lidar \
  --lidar-keyframe-interval 0.2 \
  --lidar-max-correction-translation 0.4 \
  --output results/exp_20260729_222105_353335/gnss_imu_lidar_filtered_04/trajectory_est.csv
```

Run the complete README-aligned sensor configuration:

```bash
cd /home/fizz/eVTOL/offline_fusion
./build/linux_visual/offline_fusion \
  --data /mnt/e/data/Airsim/exp_20260729_222105_353335 \
  --enable-fiveg \
  --enable-lidar \
  --output results/exp_20260729_222105_353335/all_sensors_readme_aligned/trajectory_est.csv
```

## Current Scope

- Input data defaults to `simdata_v2`.
- IMU propagation uses full preintegration with quaternion attitude and IMU bias states.
- Structured GNSS uses a simulation-specific SPP-like solver. Fixed virtual satellite
  positions are initialized from the first epoch; every epoch then solves N/E/D and
  receiver clock bias from undifferenced pseudorange and line-of-sight directions.
- Satellite weights combine squared sine elevation weighting and linearized SNR power,
  followed by per-epoch normalization and robust reweighting.
- `gps_fix` is never added as a position or velocity factor. It remains available only
  through the dataset interface and independent baseline tools.
- 5G-A factors are optional (`--enable-fiveg`) and use PRS-RSRP filtering,
  reported range standard deviations, and a robust Multi-RTT range residual.
- Visual factors are optional and require OpenCV:
  - sparse feature detection and LK tracking;
  - keyframes;
  - landmark triangulation;
  - pixel reprojection residuals in Ceres.
- Visual triangulation is initialized from the GNSS/IMU state poses and does not use
  ground-truth position or attitude.
- LiDAR factors are optional and require OpenCV FLANN:
  - voxel filtering and range filtering;
  - point-to-plane ICP with target-scan normal estimation;
  - overlap, RMSE, correction-translation, and correction-rotation gates;
  - relative translation and rotation residuals in Ceres.
- LiDAR ICP uses GNSS/IMU poses only as its initial transform and does not use ground truth.
- Camera intrinsics default to `fx=fy=3248, cx=1024, cy=768`.
- Camera translation defaults to `t_bc = (0.5, 0.0, -0.1)`.

The structured dataset README records the 2048x1536 resolution but not focal length or
FOV. A 30 s diagnostic retained 100 landmarks and 761 reprojection factors with
`fx=fy=3248`, versus 19 landmarks and 102 factors with `fx=fy=1024`; the former also
gave the slightly lower short-run trajectory RMSE (3.135 m versus 3.162 m). This is a
data-driven working value, not a replacement for exporting the exact AirSim camera
configuration.

Important limitation: the SPP-like model assumes the current synthetic satellites are
fixed for the experiment duration. It is not a real GNSS SPP/PPP implementation with
per-epoch ECEF satellite states. Carrier phase and Doppler are loaded but not yet used.
Future moving-satellite data must provide ephemeris or per-epoch satellite position and
clock information.

## Windows Baseline Build

Windows is kept as the simple no-OpenCV baseline:

```powershell
cmake -S src -B build\offline_fusion_noopencv_ninja2 `
  -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=third_party\vcpkg\scripts\buildsystems\vcpkg.cmake `
  -DOFFLINE_FUSION_USE_CERES=ON `
  -DOFFLINE_FUSION_USE_OPENCV=OFF

cmake --build build\offline_fusion_noopencv_ninja2 --config Release
```

Run:

```powershell
$env:PATH = "D:\01_research\09_项目\DNC\eVTOL\third_party\vcpkg\installed\x64-windows\bin;$env:PATH"

.\build\offline_fusion_noopencv_ninja2\offline_fusion.exe `
  --disable-visual `
  --output results\simdata_v2_ceres_rawgnss_after_cleanup\trajectory_est.csv
```

## Linux/WSL Visual Build

The full visual line is maintained under:

```text
/home/fizz/eVTOL/offline_fusion
```

Build with Ceres, SuiteSparse, and OpenCV:

```bash
cd /home/fizz/eVTOL/offline_fusion
cmake -S src -B build/linux_visual \
  -DOFFLINE_FUSION_USE_CERES=ON \
  -DOFFLINE_FUSION_USE_OPENCV=ON
cmake --build build/linux_visual -j4
```

Run:

```bash
./build/linux_visual/offline_fusion \
  --output results/simdata_v2_ceres_visual_full/trajectory_est.csv
```

Evaluate:

```bash
python3 tools/evaluate_trajectory.py \
  --estimate results/simdata_v2_ceres_visual_full/trajectory_est.csv \
  --data simdata_v2 \
  --output results/simdata_v2_ceres_visual_full

python3 tools/plot_trajectory.py \
  --estimate results/simdata_v2_ceres_visual_full/trajectory_est.csv \
  --data simdata_v2 \
  --output results/simdata_v2_ceres_visual_full
```

## Validation Snapshot

```text
raw GNSS only:
  states: 69
  RMSE 3D: 2.832882 m

IMU + gnss_raw, Windows no-OpenCV:
  states: 69
  visual_enabled: false
  initial_cost: 47.3624
  final_cost: 24.5267
  RMSE 3D: 1.387915 m

IMU + gnss_raw + visual, Linux/WSL full:
  states: 327
  visual_keyframes: 260
  visual_landmarks: 300
  visual_reprojection_factors: 2400
  initial_cost: 4834.207
  final_cost: 677.892
  RMSE 3D: 1.244245 m
```

On Linux/WSL the visual Ceres problem uses `SPARSE_SCHUR` when Ceres is built
with SuiteSparse. On Windows the current retained build disables visual factors.
