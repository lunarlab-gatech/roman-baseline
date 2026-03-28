from __future__ import annotations

from .color_utils import hsvF_to_rgb255, rgb255_to_hsvF, HSVSpace
import hashlib
from ..logger import logger
import numpy as np
from open3d.geometry import OrientedBoundingBox
import random
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation as R
import trimesh

class RerunWrapperWindow():
    def __init__(self, enable: bool):
        self.enable = enable
        self.update_frame = 0

    # ===================== Methods to Override =====================
    def _get_blueprint_part(self) -> rrb.BlueprintPart:
        raise NotImplementedError("RerunWrapperWindow should not be used directly!")

    def _get_curr_robot_name(self) -> str:
        raise NotImplementedError("RerunWrapperWindow should not be used directly!")

    # ===================== Timeline Handlers =====================
    def _update_frame_tick(self) -> None:
        rr.set_time(f"robot_{self._get_curr_robot_name()}_update_frame_tick", sequence=self.update_frame)
        rr.set_time(f"all_update_frame_tick", sequence=self.update_frame)
        self.update_frame += 1

    def update_curr_time(self, curr_time: float) -> None:
        rr.set_time(f"robot_{self._get_curr_robot_name()}_camera_frame_time", timestamp=curr_time)
        rr.set_time(f"all_camera_frame_time", timestamp=curr_time)

    # ===================== Data Logger Helpers =====================
    @staticmethod
    def extract_data_from_obb(obb: OrientedBoundingBox, box_centers: list, box_half_sizes: list, box_quats: list) -> None:
        box_centers.append(obb.center.tolist())
        box_half_sizes.append((obb.extent / 2).tolist())

        R_matrix = np.array(obb.R, copy=True)
        quat = R.from_matrix(R_matrix).as_quat()
        box_quats.append(quat.tolist())