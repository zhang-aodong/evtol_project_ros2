"""Class-wise weighted DBSCAN of semantic LiDAR points, without instance masks."""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .projection import ProjectionResult


@dataclass(frozen=True)
class VisibleTarget:
    class_id: int
    position_body_m: tuple[float, float, float]
    distance_m: float
    lidar_point_count: int
    mask_pixel_count: int
    mean_confidence: float | None
    bbox_xywh: tuple[int, int, int, int]
    point_min_body_m: tuple = ()
    point_max_body_m: tuple = ()
    raw_point_count: int = 0
    component_count: int = 0
    # Provenance for independent simulator identity auditing, never clustering.
    point_indices: tuple = ()

    def as_dict(self) -> dict:
        return {
            "class_id": self.class_id,
            "position_body_m": list(self.position_body_m),
            "distance_m": self.distance_m,
            "lidar_point_count": self.lidar_point_count,
            "mask_pixel_count": self.mask_pixel_count,
            "mean_confidence": self.mean_confidence,
            "bbox_xywh": list(self.bbox_xywh),
            "point_min_body_m": list(self.point_min_body_m),
            "point_max_body_m": list(self.point_max_body_m),
            "raw_point_count": self.raw_point_count,
            "component_count": self.component_count,
        }


def extract_visible_targets(
    points_body_xyz: np.ndarray,
    projection: ProjectionResult,
    class_mask: np.ndarray,
    confidence_mask: np.ndarray,
    target_class_ids: Sequence[int],
    min_lidar_points: int = 2,
    min_mask_pixels: int = 20,
    include_without_lidar: bool = False,
    diagnostics: dict | None = None,
    cluster_radius_m: float = 0.8,
) -> list[VisibleTarget]:
    """Merge disconnected image fragments in 3-D; reject sparse/depth noise."""
    points = np.asarray(points_body_xyz, dtype=np.float64)
    classes = np.asarray(class_mask, dtype=np.int32)
    confidence = np.asarray(confidence_mask, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_body_xyz 必须是 Nx3")
    if classes.ndim != 2 or confidence.shape != classes.shape:
        raise ValueError("class_mask/confidence_mask 必须是同尺寸 HxW 数组")
    if not np.isfinite(cluster_radius_m) or cluster_radius_m <= 0:
        raise ValueError("cluster_radius_m must be finite and positive")

    targets: list[VisibleTarget] = []
    stats_out = {"visual_components": 0, "small_components": 0,
                 "components_without_lidar": 0, "target_lidar_points": 0,
                 "depth_rejected_points": 0, "noise_points": 0, "trimmed_points": 0}
    projected_indices = np.asarray(projection.point_indices, dtype=np.int64)
    pixels = np.asarray(projection.pixels_uv, dtype=np.int32)
    depths = np.asarray(projection.depth_m)
    # Localization uses the same occlusion rule as projection diagnostics.
    if len(pixels):
        _, inverse = np.unique(pixels[:, 1]*classes.shape[1]+pixels[:, 0], return_inverse=True)
        nearest = np.full(int(inverse.max())+1, np.inf)
        np.minimum.at(nearest, inverse, depths)
        foreground = np.isfinite(depths) & (depths <= nearest[inverse] + cluster_radius_m)
        stats_out["depth_rejected_points"] = int(np.count_nonzero(~foreground))
        pixels, projected_indices = pixels[foreground], projected_indices[foreground]
    for class_id in target_class_ids:
        binary = (classes == int(class_id)).astype(np.uint8)
        component_count, components, stats = _connected_components(binary)
        if component_count <= 1:
            continue
        projected_components = components[pixels[:, 1], pixels[:, 0]]
        stats_out["visual_components"] += component_count - 1
        if int(binary.sum()) < int(min_mask_pixels):
            stats_out["small_components"] += component_count - 1
            continue
        selected = (projected_components > 0) & np.isfinite(points[projected_indices]).all(axis=1)
        member_points = points[projected_indices[selected]]
        member_indices = projected_indices[selected]
        member_components = projected_components[selected]
        stats_out["target_lidar_points"] += len(member_points)
        stats_out["components_without_lidar"] += sum(
            np.count_nonzero(member_components == i) < min_lidar_points
            for i in range(1, component_count))
        labels = _density_clusters(member_points, cluster_radius_m, min_lidar_points)
        stats_out["noise_points"] += int(np.count_nonzero(labels < 0))
        groups = [np.flatnonzero(labels == i) for i in np.unique(labels) if i >= 0]
        # Optional legacy 2-D-only display candidates have no 3-D coordinates.
        display_only = include_without_lidar and not groups
        if display_only:
            groups = [np.arange(len(member_points))]
        for group in groups:
            cluster = member_points[group]
            raw_count = len(cluster)
            if len(cluster) and not display_only:
                center = np.median(cluster, axis=0)
                distances = np.linalg.norm(cluster-center, axis=1)
                median = np.median(distances)
                cutoff = max(cluster_radius_m/2, median + 3*1.4826*np.median(abs(distances-median)))
                kept = distances <= cutoff
                stats_out["trimmed_points"] += int(np.count_nonzero(~kept))
                cluster, group = cluster[kept], group[kept]
                if len(cluster) < min_lidar_points:
                    stats_out["noise_points"] += len(cluster)
                    continue
            ids = np.unique(member_components[group]) if len(group) else np.arange(1, component_count)
            boxes = stats[ids]
            x, y = boxes[:, :2].min(axis=0)
            right, bottom = (boxes[:, :2]+boxes[:, 2:4]).max(axis=0)
            width, height, area = int(right-x), int(bottom-y), int(boxes[:, 4].sum())
            if area < min_mask_pixels:
                stats_out["small_components"] += len(ids)
                continue
            position = np.mean(cluster, axis=0) if len(cluster) and not display_only else np.full(3, np.nan)
            component_confidence = confidence[y:y+height, x:x+width][
                np.isin(components[y:y+height, x:x+width], ids)]
            targets.append(
                VisibleTarget(
                    class_id=int(class_id),
                    position_body_m=tuple(float(value) for value in position),
                    distance_m=(
                        float(np.linalg.norm(position))
                        if len(cluster) and not display_only else float("nan")
                    ),
                    lidar_point_count=int(len(cluster)),
                    mask_pixel_count=area,
                    mean_confidence=(float(np.mean(component_confidence[np.isfinite(component_confidence)]))
                                     if np.isfinite(component_confidence).any() else None),
                    bbox_xywh=(x, y, width, height),
                    point_min_body_m=tuple(cluster.min(axis=0)) if len(cluster) else (),
                    point_max_body_m=tuple(cluster.max(axis=0)) if len(cluster) else (),
                    raw_point_count=raw_count,
                    component_count=len(ids),
                    point_indices=tuple(int(v) for v in member_indices[group]),
                )
            )
    targets.sort(
        key=lambda item: item.distance_m
        if np.isfinite(item.distance_m) else float("inf")
    )
    if diagnostics is not None:
        diagnostics.update(stats_out)
    return targets


def _density_clusters(points, radius, min_points):
    """Voxel-weighted DBSCAN. Bounded local voxel density avoids dense N² graphs."""
    from scipy.spatial import cKDTree
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import connected_components
    if not len(points):
        return np.empty(0, dtype=int)
    _, inverse, counts = np.unique(np.floor(points/(radius/3)).astype(np.int64),
                                   axis=0, return_inverse=True, return_counts=True)
    reps = np.column_stack([np.bincount(inverse, weights=points[:, i])/counts for i in range(3)])
    neighbours = cKDTree(reps).query_ball_point(reps, radius)
    core = np.array([counts[ids].sum() >= min_points for ids in neighbours])
    labels = np.full(len(reps), -1, dtype=int)
    core_ids = np.flatnonzero(core)
    if len(core_ids):
        lookup = np.full(len(reps), -1, dtype=int)
        lookup[core_ids] = np.arange(len(core_ids))
        rows, cols = [], []
        for i in core_ids:
            links = [j for j in neighbours[i] if core[j]]
            rows.extend([lookup[i]]*len(links))
            cols.extend(lookup[links])
        graph = csr_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(core_ids), len(core_ids)))
        _, labels[core_ids] = connected_components(graph, directed=False)
        border = np.flatnonzero(~core)
        distances, nearest = cKDTree(reps[core_ids]).query(reps[border], distance_upper_bound=radius)
        accepted = np.isfinite(distances)
        labels[border[accepted]] = labels[core_ids[nearest[accepted]]]
    return labels[inverse]



def _connected_components(binary: np.ndarray) -> tuple[int, np.ndarray, np.ndarray]:
    """Use compiled connected-component labelling; never scan full images in Python."""
    from scipy import ndimage
    components, count = ndimage.label(binary, structure=np.ones((3, 3), dtype=np.uint8))
    areas = np.bincount(components.ravel())
    stats = np.zeros((count + 1, 5), dtype=np.int32)
    for component_id, region in enumerate(ndimage.find_objects(components), start=1):
        y, x = region
        stats[component_id] = (x.start, y.start, x.stop-x.start, y.stop-y.start, areas[component_id])
    return count + 1, components, stats
