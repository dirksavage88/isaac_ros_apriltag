# Isaac ROS Apriltag

<div align="center"><img src="resources/isaac_ros_apriltag_sample_crop.jpg" width="400px"/></div>
    
## Overview
This ROS2 node uses the NVIDIA GPU-accelerated AprilTags library to detect AprilTags in images and publishes their poses, IDs, and additional metadata. This has been tested on ROS2 (Foxy) and should build and run on x86_64 and aarch64 (Jetson). It is modeled after and comparable to the ROS2 node for [CPU AprilTags detection](https://github.com/christianrauch/apriltag_ros.git).

For more information on the Isaac GEM that this node is based off of, see the latest Isaac SDK documentation [here](https://docs.nvidia.com/isaac/isaac/packages/fiducials/doc/apriltags.html).

For more information on AprilTags themselves, including the paper and the reference CPU implementation, click [here](https://april.eecs.umich.edu/software/apriltag.html).

## System Requirements
This Isaac ROS package is designed and tested to be compatible with ROS2 Foxy on Jetson hardware.
### Jetson
- AGX Xavier or Xavier NX
- JetPack 4.6

### x86_64
- CUDA 10.2+ supported discrete GPU
- VPI 1.1.11
- Ubuntu 18.04+

**Note:** For best performance on Jetson, ensure that power settings are configured appropriately ([Power Management for Jetson](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0EUHA)).

### Docker
Precompiled ROS2 Foxy packages are not available for JetPack 4.6 (based on Ubuntu 18.04 Bionic). You can either manually compile ROS2 Foxy and required dependent packages from source or use the Isaac ROS development Docker image from [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common).

You must first install the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to make use of the Docker container development/runtime environment.

Configure `nvidia-container-runtime` as the default runtime for Docker by editing `/etc/docker/daemon.json` to include the following:
```
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```
and then restarting Docker: `sudo systemctl daemon-reload && sudo systemctl restart docker`

Run the following script in `isaac_ros_common` to build the image and launch the container:

`$ scripts/run_dev.sh <optional path>`

You can either provide an optional path to mirror in your host ROS workspace with Isaac ROS packages, which will be made available in the container as `/workspaces/isaac_ros-dev`, or you can setup a new workspace in the container.

## Docker build notes
### 1) Example docker run command with proper bind mounts to cuda libs and video device mount (note you will need to change which workspace you mount depending on where you have your ros repos):
```
sudo docker run --runtime nvidia --rm --privileged --net=host -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v /workspaces/isaac_ros-dev/:/workspaces/isaac_ros-dev/ -v /usr/local/cuda-10.2/targets/aarch64-linux/lib/:/usr/local/cuda-10.2/targets/aarch64-linux/lib/ -v /tmp/argus_socket:/tmp/argus_socket -v /usr/src/jetson_multimedia_api/:/usr/src/jetson_multimedia_api/ --device /dev/video0 -it isaac_ros_dev-aarch64:latest /bin/bash
```

#### If running locally on the Jetson nano with display connected you need to include the environment variable to the local display
```
sudo docker run --runtime nvidia --rm --privileged --net=host -v /tmp/.X11-unix/:/tmp/.X11-unix/ -v /workspaces/isaac_ros-dev/:/workspaces/isaac_ros-dev/ -v /usr/local/cuda-10.2/targets/aarch64-linux/lib/:/usr/local/cuda-10.2/targets/aarch64-linux/lib/ -v /tmp/argus_socket:/tmp/argus_socket -v /usr/src/jetson_multimedia_api/:/usr/src/jetson_multimedia_api/ --device /dev/video0 -e DISPLAY=$DISPLAY -it isaac_ros_dev-aarch64:latest /bin/bash
```

#### You will need to install jetson multimedia api packages on host with if not already installed via factory or user OS flash
```
sudo apt install nvidia-l4t-jetson-multimedia-api
```

### 2) I don't remember why argus socket bind is required, but without it I think running the argus camera node failed. Here is an example to run argus with a raspberry pi cam:
```
ros2 run isaac_ros_argus_camera_mono isaac_ros_argus_camera_mono --ros-args -r /image_raw:=/image_rect -p device:=0 -p sensor:=4 -p output_encoding:=mono8 -p camera_info_url:=file:///workspaces/isaac_ros-dev/ros_ws/isaac_ros_apriltag/config/rpi_cam.yaml
```

### 3) Another dependency is nvidia-cudnn8. This is needed for the camera_calibration node. On the host install locally and the cuda libs should be found by binding the libs workspaces in the docker run example above
```
sudo apt install nvidia-cudnn8
```

### 4) In your binded ros2 workspace, you may find target export name errors, see the following for a fix:
```
cd ros_ws
sudo chown -R admin:admin .
```

### 5) If you get cmake rosidl_generate_interfaces errors, you may need to switch which python version via update alternatives:
```
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2

sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1

sudo update-alternatives --config python
```

### 6) If you have issues with the following error:

   ```
   docker: Error response from daemon: pull access denied for isaac_ros_dev-aarch64, repository does not exist or may require 'docker login': denied: requested access to the resource is denied.
   ```

   #### Try to run the sample nvidia container runtime workload: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html
   ```
   sudo docker run --rm --runtime=nvidia ubuntu nvidia-smi
   ```

### 8) If you OCI runtime failures, it could be multiple issues, see these fixes: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/issues/5

### 9) Note that there are known issues with the navargs-daemon. I have found that even restarting the service does not solve nvidia argus node failures, but rebooting the jetson does. See: https://forums.developer.nvidia.com/t/nvargus-errors-timeout-and-inevitable-failure-happens-on-4x-different-nanos/175487

### Package Dependencies
- [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [isaac_ros_image_pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline)
- [image_common](https://github.com/ros-perception/image_common.git)
- [vision_cv](https://github.com/ros-perception/vision_opencv.git)
- [OpenCV 4.5+](https://opencv.org/)

**Note:** `isaac_ros_common` is used for running tests and/or creating a development container. It also contains VPI Debian packages that can be installed natively on a development machine without the container.

## Quickstart
1. Create a ROS2 workspace if one is not already prepared:  
`mkdir -p your_ws/src`  
**Note:** The workspace can have any name; the quickstart assumes you name it `your_ws`.
2. Clone this package repository to `your_ws/src/isaac_ros_apriltag`. Check that you have [Git LFS](https://git-lfs.github.com/) installed before cloning to pull down all large files.  
`sudo apt-get install git-lfs`  
`cd your_ws/src && git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag`
3. Build and source the workspace:  
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
4. (Optional) Run tests to verify complete and correct installation:  
`colcon test`
5. Start `isaac_ros_apriltag` using the prebuilt executable:  
`ros2 run isaac_ros_apriltag isaac_ros_apriltag`
6. In a separate terminal, spin up a **calibrated** camera publisher to `/image_rect` and `/camera_info` using any package (for example, `v4l2_camera`):  
`ros2 run v4l2_camera v4l2_camera_node --ros-args -r /image_raw:=/image_rect`
7. Observe the AprilTag detection output `/tag_detections` on a separate terminal with the command:   
`ros2 topic echo /tag_detections`

### Configuration
You will need to calibrate the intrinsics of your camera if you want the node to determine 3D poses for tags instead of just detection and corners as 2D pixel coordinates. See [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html) for more details.

### Replacing `apriltag_ros` with `isaac_ros_apriltag`
1. Add a dependency on `isaac_ros_apriltag` to `your_package/package.xml` and `your_package/CMakeLists.txt`. The original `apriltag_ros` dependency may be removed entirely.
2. Change the package and plugin names in any `*.launch.py` launch files to use `isaac_ros_apriltag` and `AprilTagNode`, respectively.

## See Also
- `isaac_ros_image_pipeline`: Accelerated metapackage offering similar functionality to the standard CPU-based `image_pipeline` metapackage
- `isaac_ros_common`: Utilities for robust ROS2 testing, in conjunction with `launch_test`

# Isaac ROS Apriltag Pipeline Tutorial
## Objective
This tutorial will help you quickly run and experiment with the full Isaac ROS Apriltag pipeline, from camera frames to tag detections.

## Tutorial
1. Complete the Quickstart steps above.
2. Connect a compatible camera to your Jetson and set up the camera publisher stream. Your camera vendor may offer a specific ROS2-compatible camera driver package. Alternatively, many generic cameras are compatible with the `v4l2_camera` package.  
**Important:** Ensure that the camera stream publishes `Image` and `CameraInfo` pairs to the topics `/image_raw` and `/camera_info`, respectively.
3. Ensure that your workspace has been built and sourced, if you have not done so already:  
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
4. Finally, launch the pre-composed pipeline launchfile:  
`ros2 launch isaac_ros_apriltag isaac_ros_apriltag_pipeline.launch.py`

Detections will show up at `/tag_detections`.

## Next Steps
Now that you have successfully launched the full Isaac ROS Apriltag pipeline, you can easily adapt the provided launchfile to integrate with your existing ROS2 environment. 

Alternatively, since the `AprilTagNode` is provided as a ROS2 Component, you can also compose the accelerated Apriltag processor directly into an existing executable.

# Package Reference
## `isaac_ros_apriltag`
### Overview
The `isaac_ros_apriltag` package offers functionality for detecting poses from AprilTags in the frame. It largely replaces the `apriltag_ros` package, though an included dependency on the `ImageFormatConverterNode` plugin of the `isaac_ros_image_proc` package also functions as a way to replace the CPU-based image format conversion in `cv_bridge`.
### Available Components
| Component      | Topics Subscribed                                                  | Topics Published                                                       | Parameters                                                                                                                                                                                                               |
| -------------- | ------------------------------------------------------------------ | ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `AprilTagNode` | `camera/image_rect`, `camera/camera_info`: The input camera stream | `tag_detections`: The detection message array <br> `tf`: The tag poses | `family`: The tag family for the detector (this value can only be `36h11` at this time) <br> `size`: The tag edge size in meters, assuming square markers <br> `max_tags`: The maximum number of tags to be detected, which is 20 by default |


# Updates

| Date | Changes |
| -----| ------- |
| 2021-10-20 | Migrated to [NVIDIA-ISAAC-ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag)  |
| 2021-08-11 | Initial release to [NVIDIA-AI-IOT](https://github.com/NVIDIA-AI-IOT/isaac_ros_apriltag) |
