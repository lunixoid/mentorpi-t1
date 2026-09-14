"""PointCloud2 → numpy xyz (SD012 D3.2.2)."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

POINTFIELD_FLOAT32 = 7

DETAIL_CLOUD_LAYOUT_UNSUPPORTED = "cloud layout unsupported"

# Minimum points for later floor-plane gates (D3.2.7); not enforced here.
MIN_FLOOR_POINTS = 500


class CloudLayoutError(ValueError):
    """PointCloud2 layout cannot be parsed as xyz float32."""

    def __init__(self, detail: str = DETAIL_CLOUD_LAYOUT_UNSUPPORTED) -> None:
        super().__init__(detail)
        self.detail = detail


def pointcloud2_xyz(msg: Any) -> np.ndarray:
    """Parse a sensor_msgs PointCloud2-like message into an (N, 3) float32 array."""
    offsets: dict[str, int] = {}
    for field in msg.fields:
        if field.name not in ("x", "y", "z"):
            continue
        if field.datatype != POINTFIELD_FLOAT32:
            raise CloudLayoutError(DETAIL_CLOUD_LAYOUT_UNSUPPORTED)
        offsets[field.name] = field.offset

    if set(offsets) != {"x", "y", "z"}:
        raise CloudLayoutError(DETAIL_CLOUD_LAYOUT_UNSUPPORTED)

    n_points = int(msg.width) * int(msg.height)
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)

    point_step = int(msg.point_step)
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": point_step,
        }
    )
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=n_points)
    xyz = np.column_stack((raw["x"], raw["y"], raw["z"]))
    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def _is_pointcloud2_like(cloud: Any) -> bool:
    return (
        hasattr(cloud, "fields")
        and hasattr(cloud, "data")
        and hasattr(cloud, "point_step")
        and hasattr(cloud, "width")
        and hasattr(cloud, "height")
    )


def clouds_to_xyz(clouds: Sequence[Any]) -> np.ndarray:
    """Concatenate clouds into one (N, 3) array (PointCloud2 or pre-parsed Nx3)."""
    if not clouds:
        return np.empty((0, 3), dtype=np.float32)

    parts: list[np.ndarray] = []
    for cloud in clouds:
        if isinstance(cloud, np.ndarray):
            if cloud.ndim != 2 or cloud.shape[1] != 3:
                raise CloudLayoutError(DETAIL_CLOUD_LAYOUT_UNSUPPORTED)
            parts.append(np.asarray(cloud, dtype=np.float32))
        elif _is_pointcloud2_like(cloud):
            parts.append(pointcloud2_xyz(cloud))
        else:
            raise CloudLayoutError(DETAIL_CLOUD_LAYOUT_UNSUPPORTED)

    return np.concatenate(parts, axis=0)


def count_cloud_points(clouds: Sequence[Any]) -> int | None:
    """Return total finite xyz points, or None if any cloud layout is unsupported."""
    if not clouds:
        return 0
    try:
        return len(clouds_to_xyz(clouds))
    except CloudLayoutError:
        return None
