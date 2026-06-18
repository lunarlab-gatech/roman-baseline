# ROMAN
Our fork of the [ROMAN](https://github.com/mit-acl/roman) repository for evaluation as a baseline running on the HERCULES dataset.

## Install

### Docker Setup

Make sure to install:
- [Docker](https://docs.docker.com/engine/install/ubuntu/)

Then, clone this repository into a desired location on your computer.

After that, navigate to the `docker` directory. Log in to the user that you want the docker file to create in the container. 

Edit the `enter_container.sh` script with the following paths:
- `DATA_DIR=`: The directory where the HERCULES dataset is located
- `REPO_DIR=`: The directory of this repository

Now, run the following commands:
```
build_container.sh
run_container.sh
```

The rest of this README **assumes that you are inside the Docker container**. For easier debugging and use, its highly recommended to install the [VSCode Docker extension](https://code.visualstudio.com/docs/containers/overview), which allows you to start/stop the container and additionally attach VSCode to the container by right-clicking on the container and selecting `Attach Visual Studio Code`. If that isn't possible, you can re-enter the container running the following command:
```
enter_container.sh
```

### ROMAN Install

Next, install ROMAN by running the following command from the root folder of this repository:
```
./install.sh
```

Note that if you needed to make a new Docker container but kept the repository, you should instead run `reinstall.sh` instead of `install.sh`.


Finally, run the following to fix `Could not load the Qt platform plug "xcb"` bug:
```
mv ~/.local/lib/python3.10/site-packages/cv2/qt ~/.local/lib/python3.10/site-packages/cv2/qt.bak
```

## Experiments

### HERCULES

First, edit the absolute paths in the files in `params/hercules`. Then, run the following command to run this demo:

```
export YOLO_VERBOSE=False
export TYPEGUARD_DISABLE=1
python3 research/run_slam.py -p params/<sequence_directory>/
```

#### WandB Sweeps

To run a wandb sweep, use the following command to initialize the sweep:
```
wandb sweep --entity <wandb_entity_name> --project <project_name> <sweep YAML config>
```

This command will give a new command that will execute the sweep.

#### Note on Parameters (May be slightly outdated)

In order to adapt ROMAN to work successfully on HERCULES, two types of parameters were changed:
- 1. Robot parameters, or those that would always have to change due to differences in the robots we are using. 
- 2. Tunable parameters, or those that don't fall in the category above.

For a fair comparison with ROMAN, ideally we only change parameters in category 1. However, due to the wildly different "Australian Environment", we found that it was necessary to change some parameters in 2 to successfully find a map alignment. Thus, below we document all parameters that were changed in both categories 1 & 2; 1 so that its easy to see for the future what we would need to change to apply other experiments, and 2 so that the reasons for these changes can be well documented:

Category 1:
```
data.yaml: runs, run_env, img_data, depth_data, pose_data
fastsam.yaml: weights_path, yolo_weights_path, depth_scale, depth_data_type
gt_pose.yaml: path, csv_options, time_tol, interp, causal
```

Category 2:
```
fastsam.yaml: voxel_size, max_depth
mapper.yaml: iou_voxel_size, segment_voxel_size
submap_align.yaml: submap_radius, submap_center_dist, submap_center_time, sigma, epsilon
offline_rpgo.yaml: gnc_inlier_threshold
```

For reasons for changes in category 2, see the corresponding .yaml files.