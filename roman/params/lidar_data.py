from __future__ import annotations

###########################################################
#
# lidar_data.py
#
# Minimal loader for per-scan LiDAR point clouds stored as individual
# .npy files (one file per scan, named "<timestamp>.npy"), matching the
# on-disk layout produced by extract_data_ROMAN.py-style preprocessing
# (e.g. HERCULES, GrAco "files_for_roman_baseline/<robot>/lidar/").
#
###########################################################

import numpy as np
from pathlib import Path
from robotdatapy.data.robot_data import NoDataNearTimeException


class LidarNpyData:
    """ Loads a directory of per-scan LiDAR point cloud .npy files. """

    def __init__(self, times: np.ndarray, files: list, time_tol: float = 0.5):
        order = np.argsort(times)
        self.times = np.asarray(times, dtype=np.float64)[order]
        self.files = [files[i] for i in order]
        self.time_tol = time_tol

    @classmethod
    def from_npy_dir(cls, dir_path: str, time_tol: float = 0.5) -> LidarNpyData:
        dir_path = Path(dir_path)
        files = sorted(dir_path.glob('*.npy'))
        assert len(files) > 0, f"No .npy files found in {dir_path}"
        times = [float(f.stem) for f in files]
        return cls(times, files, time_tol=time_tol)

    def _nearest_idx(self, t: float) -> int:
        idx = int(np.argmin(np.abs(self.times - t)))
        if abs(self.times[idx] - t) > self.time_tol:
            raise NoDataNearTimeException(t_desired=t, t_closest=self.times[idx])
        return idx

    def nearest_time(self, t: float) -> float:
        return self.times[self._nearest_idx(t)]

    def point_cloud(self, t: float) -> np.ndarray:
        """ (N, 3) point cloud nearest to time t, in the frame the .npy files were saved in
        (no axis-convention conversion is applied here; that is handled by T_base_lidar). """
        idx = self._nearest_idx(t)
        return np.load(self.files[idx]).astype(np.float64)[:, :3]
