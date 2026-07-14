###########################################################
#
# run.py
#
# A class for processing ROMAN mapping
#
# Authors: Mason Peterson, Yulun Tian, Lucas Jia
#
# Dec. 21, 2024
#
###########################################################

import numpy as np

from dataclasses import dataclass
import time
from copy import deepcopy

from robotdatapy.data.robot_data import NoDataNearTimeException

from roman.object.segment import Segment
from roman.map.mapper import Mapper
from roman.map.fastsam_wrapper import FastSAMWrapper
from roman.params.system_params import SystemParams
from ..rerun_wrapper.rerun_wrapper_window_map import RerunWrapperWindowMap
from ..logger import logger

@dataclass
class ProcessingTimes:
    fastsam_times: list
    map_times: list
    total_times: list

class ROMANMapRunner:
    def __init__(self, system_params: SystemParams, rerun_viewer: RerunWrapperWindowMap, verbose=False):
        self.system_params = system_params
        self.data_params = system_params.data_params
        self.fastsam_params = system_params.fastsam_params
        self.mapper_params = system_params.mapper_params

        if verbose: print("Extracting time range...")
        self.time_range = self.data_params.time_range

        if verbose: 
            print("Loading image data...")
            print(f"Time range: {self.time_range}")

        self.img_data = self.data_params.load_img_data()
        if verbose:
            self.t0 = self.img_data.t0
            self.tf = self.img_data.tf

        # Detect point cloud source: LiDAR or depth
        self.use_lidar = self.data_params.use_lidar

        if self.use_lidar:
            if verbose: print("Loading LiDAR data...")
            self.lidar_data = self.data_params.load_lidar_data()
            T_base_lidar = self.data_params.pose_data_params.T_base_lidar
            if T_base_lidar is None:
                raise ValueError("T_base_lidar must be provided in data.yaml when using LiDAR data.")
            # Assumes T_camera_flu is also T_camera_base (i.e. the FLU frame is co-located
            # with the odometry base frame), so it can be composed directly with T_base_lidar.
            self.T_camera_lidar = self.data_params.pose_data_params.T_camera_flu @ T_base_lidar
            self.depth_data = None
            cam_params = self.img_data.camera_params
        else:
            if verbose: print("Loading depth data for time range {}...".format(self.time_range))
            self.depth_data = self.data_params.load_depth_data()
            cam_params = self.depth_data.camera_params

        if verbose: print("Loading pose data...")
        self.camera_pose_data = self.data_params.load_pose_data()

        if verbose: print("Setting up FastSAM...")
        self.fastsam = FastSAMWrapper.from_params(self.fastsam_params, cam_params, self.system_params)

        if verbose: print("Setting up mapper...")
        self.mapper = Mapper(self.mapper_params, self.img_data.camera_params)
        if self.data_params.pose_data_params.T_camera_flu is not None:
            self.mapper.set_T_camera_flu(self.data_params.pose_data_params.T_camera_flu)
        
        self.verbose = verbose
        self.processing_times = ProcessingTimes([], [], [])

    def times(self):
        return np.arange(self.t0, self.tf, self.data_params.dt)

    def update(self, t: float) -> None: 
        t0 = self.img_data.t0
        tf = self.img_data.tf

        update_t0 = time.time()

        img_time, observations, pose_odom_camera, img, depth_img = self.update_fastsam(t)
        update_t1 = time.time()
        if observations is not None and pose_odom_camera is not None and img is not None:
            self.update_segment_track(img_time, observations, pose_odom_camera, img, depth_img)
        
        update_t2 = time.time()
        self.processing_times.map_times.append(update_t2 - update_t1)
        self.processing_times.fastsam_times.append(update_t1 - update_t0)
        self.processing_times.total_times.append(update_t2 - update_t0)

    def update_fastsam(self, t):

        try:
            img_t = self.img_data.nearest_time(t)
        except NoDataNearTimeException as e:
            logger.info(f"[red]NoDataNearTimeException[/red]: No time within threshold of time {img_t}.")
            return None, None, None, None, None
        
        try:
            img = self.img_data.img(img_t)
        except NoDataNearTimeException as e:
            logger.info(f"[red]NoDataNearTimeException[/red]: No image data within threshold of time {img_t}.")
            return None, None, None, None, None
        
        img_depth = None
        lidar_pc_camera = None

        if self.use_lidar:
            try:
                lidar_pc_camera = self._get_lidar_pc_at_time(float(img_t))
            except NoDataNearTimeException as e:
                logger.info(f"[red]NoDataNearTimeException[/red]: No LiDAR data within threshold of time {img_t}.")
                return None, None, None, None, None
        else:
            try:
                img_depth = self.depth_data.img(img_t)
            except NoDataNearTimeException as e:
                logger.info(f"[red]NoDataNearTimeException[/red]: No depth data within threshold of time {img_t}.")
                return None, None, None, None, None

        try:
            pose_odom_camera = self.camera_pose_data.T_WB(img_t)
        except NoDataNearTimeException as e:
            logger.info(f"[red]NoDataNearTimeException[/red]: No pose data within threshold of time {img_t}.")
            return None, None, None, None, None

        observations = self.fastsam.run(float(img_t), pose_odom_camera, img,
                                        img_depth=img_depth, lidar_pc_camera=lidar_pc_camera)
        return img_t, observations, pose_odom_camera, img, img_depth

    def _get_lidar_pc_at_time(self, t: float) -> np.ndarray:
        """Get the LiDAR point cloud nearest to time t, transformed to the camera optical frame.

        Args:
            t: Query time (seconds).

        Returns:
            (M, 3) array of points in the camera optical frame.

        Raises:
            NoDataNearTimeException: If no LiDAR scan is within tolerance of t.
        """
        lidar_t = self.lidar_data.nearest_time(t)
        pc = self.lidar_data.point_cloud(lidar_t)

        ones = np.ones((pc.shape[0], 1), dtype=pc.dtype)
        pc_h = np.hstack([pc, ones])
        pc_camera = (self.T_camera_lidar @ pc_h.T).T[:, :3]

        return pc_camera

    def update_segment_track(self, t, observations, pose_odom_camera, img, depth_img) -> None: 

        # Create the segmentation image
        height = self.data_params.img_data_params.height
        width = self.data_params.img_data_params.width
        seg_img = np.zeros((len(observations), height, width), dtype=np.uint16)
        for i, obs in enumerate(observations):
            mask = obs.mask
            seg_img[i] = np.multiply(mask.astype(np.uint16), np.full((height, width), i+1, dtype=np.uint16))

        if len(observations) > 0:
            self.mapper.update(t, pose_odom_camera, observations)
        else:
            self.mapper.poses_flu_history.append(pose_odom_camera @ self.mapper._T_camera_flu)
            self.mapper.times_history.append(t)
