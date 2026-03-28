from __future__ import annotations

from .data_params import DataParams, PoseDataGTParams
from .fastsam_params import FastSAMParams
from .mapper_params import MapperParams
from .offline_rpgo_params import OfflineRPGOParams
import os
from pathlib import Path
from pydantic import BaseModel, model_validator
from robotdatapy.data.pose_data import PoseData
from roman.utils import expandvars_recursive
from .submap_align_params import SubmapAlignParams
from .path_params import PathParams
import yaml

class SystemParams(BaseModel):
    """
    Wrapper class that holds sub-classes for each of the sub-system parameters.
    
    Args:
        data_params (DataParams): Parameters for data loading.
        fastsam_params (FastSAMParams): Parameters for FastSAM & YOLO.
        mapper_params (MapperParams): Parameters for ROMAN's mapper.
        offline_rpgo_params (OfflineRPGOParams): Parameters for Kimera-RPGO.
        submap_align_params (SubmapAlignParams): Parameters for Submap alignment via CLIPPER.
        gt_file (Path | None): Optional path to ground truth pose file.
        num_req_assoc (int): Number of required associations for merging nodes.
    """

    path_params: PathParams
    data_params: DataParams
    pose_data_gt_params: PoseDataGTParams
    fastsam_params: FastSAMParams
    mapper_params: MapperParams
    offline_rpgo_params: OfflineRPGOParams
    submap_align_params: SubmapAlignParams
    num_req_assoc: int
    enable_rerun_viz: bool
    seed: int

    @classmethod
    def from_param_dir(cls, path: str) -> SystemParams:
        params_path = Path(path)

        with open(params_path / "system_params.yaml") as f:
            data = yaml.safe_load(f)
        num_req_assoc = data['num_req_assoc']
        enable_rerun_viz = data['enable_rerun_viz']
        seed = data['seed']

        path_params = PathParams.from_dict(data['path_params'])
        data_params = DataParams.from_yaml(params_path / "data.yaml", path_params)
        pose_data_gt_params = PoseDataGTParams.from_yaml(params_path / "gt_pose.yaml", path_params)
        fastsam_params = FastSAMParams.from_yaml(params_path / "fastsam.yaml")
        mapper_params = MapperParams.from_yaml(params_path / "mapper.yaml")
        offline_rpgo_params = OfflineRPGOParams.from_yaml(params_path / "offline_rpgo.yaml") 
        submap_align_params = SubmapAlignParams.from_yaml(params_path / "submap_align.yaml")
        
        return cls(path_params=path_params,
                   data_params=data_params, 
                   pose_data_gt_params=pose_data_gt_params,
                   fastsam_params=fastsam_params, 
                   mapper_params=mapper_params,
                   offline_rpgo_params=offline_rpgo_params, 
                   submap_align_params=submap_align_params, 
                   num_req_assoc=num_req_assoc,
                   enable_rerun_viz=enable_rerun_viz,
                   seed=seed)