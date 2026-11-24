"""Trajectory scores at common truth epochs. Distances are in metres."""
import numpy as np


def supported(query, samples, max_gap):
    hi = np.searchsorted(samples, query)
    left = np.clip(hi - 1, 0, len(samples) - 1)
    right = np.clip(hi, 0, len(samples) - 1)
    return ((query >= samples[0]) & (query <= samples[-1]) &
            ((query == samples[right]) | (samples[right] - samples[left] <= max_gap)))


def relative_rmse(times, errors, matched, blocks, horizon):
    # Interpolate the error at the earlier endpoint on its own uninterrupted segment.
    earlier = times - horizon
    valid = matched & supported(earlier, times[matched], 0.25)
    start_indices = np.clip(np.searchsorted(times, earlier), 0, len(times) - 1)
    valid &= (blocks == blocks[start_indices])
    if not valid.any():
        return None
    previous = np.column_stack([np.interp(earlier[valid], times[matched], axis[matched])
                                for axis in errors.T])
    return float(np.sqrt(np.mean(np.sum((errors[valid] - previous)**2, axis=1))))


def summarize(truth, estimate, local_vio, max_gap_s=0.6):
    """Each input is (monotonic int64 ns, Nx3 positions, N ENU yaw angles)."""
    truth_ns, reference, truth_yaw = truth
    est_ns, position, est_yaw = estimate
    for stamps, points, yaw in (truth, estimate):
        if len(stamps) < 2 or np.any(np.diff(stamps) <= 0):
            raise ValueError("Need at least two strictly increasing samples per trajectory")
        if not np.isfinite(points).all() or not np.isfinite(yaw).all():
            raise ValueError("Non-finite trajectory")
    times = (truth_ns - truth_ns[0]) * 1e-9
    epochs = (est_ns - truth_ns[0]) * 1e-9
    matched = supported(times, epochs, max_gap_s)
    if matched.sum() < 2:
        raise ValueError("Insufficient common time coverage")
    sample = np.column_stack([np.interp(times, epochs, axis) for axis in position.T])
    if local_vio:
        i = np.flatnonzero(matched)[0]
        angle = truth_yaw[i] - np.interp(times[i], epochs, np.unwrap(est_yaw))
        c, s = np.cos(angle), np.sin(angle)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        sample = (sample - sample[i]) @ rotation.T + reference[i]
    errors = sample - reference
    distance = np.linalg.norm(errors[matched], axis=1)
    # Split on missing estimates and on gaps in either original stream.
    boundary = np.r_[True, ~matched[:-1] | (np.diff(times) > 0.25)]
    for after_gap in epochs[1:][np.diff(epochs) > max_gap_s]:
        i = np.searchsorted(times, after_gap)
        if i < len(times):
            boundary[i] = True
    blocks = np.cumsum(boundary)
    kind = "aligned" if local_vio else "absolute"
    return {
        "alignment": "first common pose: translation and yaw; unit scale" if local_vio else "session map_enu",
        "truth_epochs": len(times), "estimate_epochs": len(epochs),
        "matched_epochs": int(matched.sum()), "coverage": float(matched.mean()),
        f"{kind}_3d_rmse_m": float(np.sqrt(np.mean(distance**2))),
        f"{kind}_3d_p95_m": float(np.percentile(distance, 95)),
        "rpe_1s_rmse_m": relative_rmse(times, errors, matched, blocks, 1.),
        "rpe_5s_rmse_m": relative_rmse(times, errors, matched, blocks, 5.),
    }
