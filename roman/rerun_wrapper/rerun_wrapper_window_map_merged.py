from __future__ import annotations

import copy
from .color_utils import hsvF_to_rgb255, rgb255_to_bgr255, HSVSpace
import cv2
import hashlib
from ..logger import logger
import numpy as np
from open3d.geometry import OrientedBoundingBox
from ..params.data_params import ImgDataParams
import random
import rerun as rr
import rerun.blueprint as rrb
from .rerun_wrapper_window import RerunWrapperWindow
from roman.object.segment import Segment
from roman.params.fastsam_params import FastSAMParams
from scipy.spatial.transform import Rotation as R
import trimesh


class RerunWrapperWindowMapMerged(RerunWrapperWindow):
    def __init__(self, enable: bool, robot_name_0: str, robot_name_1: str, fastsam_params: FastSAMParams):
        super().__init__(enable)
        self.robot_name_0: str = robot_name_0
        self.robot_name_1: str = robot_name_1
        self.fastsam_params: FastSAMParams = fastsam_params
        self.id_to_color_mapping: dict = dict()

    # ===================== Methods to Override =====================
    def _get_blueprint_part(self) -> rrb.BlueprintPart:
        # Create the views
        graph_view_0 = rrb.GraphView(name="Graph", origin=f'/graph/{self.robot_name_0}')
        graph_view_1 = rrb.GraphView(name="Graph", origin=f'/graph/{self.robot_name_1}')
        world_view = rrb.Spatial3DView(name="World", origin=f'/world')

        image_view_0 = rrb.Spatial2DView(name="Image", origin=f'/world/{self.robot_name_0}/camera/image')
        depth_view_0 = rrb.Spatial2DView(name="Depth", origin=f'/world/{self.robot_name_0}/camera/depth')

        image_view_1 = rrb.Spatial2DView(name="Image", origin=f'/world/{self.robot_name_1}/camera/image')
        depth_view_1 = rrb.Spatial2DView(name="Depth", origin=f'/world/{self.robot_name_1}/camera/depth')

        seg_view_0 = rrb.Spatial2DView(name="Segmentation Mask", origin=f'/world/{self.robot_name_0}/camera/segmentation')
        seg_view_1 = rrb.Spatial2DView(name="Segmentation Mask", origin=f'/world/{self.robot_name_1}/camera/segmentation')

        # Create the tab
        robot_0_vert = rrb.Vertical(graph_view_0, image_view_0, depth_view_0, seg_view_0)
        robot_1_vert = rrb.Vertical(graph_view_1, image_view_1, depth_view_1, seg_view_1)
        tab_name: str = f"World Map"
        return rrb.Horizontal(robot_0_vert, world_view, robot_1_vert, name=tab_name)
    
