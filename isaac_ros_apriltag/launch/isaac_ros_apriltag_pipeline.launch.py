# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
import launch
import yaml
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# detect all 36h11 tags
cfg_36h11 = {
        'image_transport':'raw', 
        'family':'36h11',
        'size':0.162
    }

def generate_launch_description():

    cam_yaml = '/isaac_ros_apriltag' + '/config' + '/rpi_cam.yaml'
    cam_yaml_abs = os.path.abspath(os.getcwd())
    cat_path = 'file://' + cam_yaml_abs + cam_yaml
    
    rectify_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify_node',
    )

    rectify_container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[rectify_node],
        output='screen'
    )
    composable_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='isaac_ros::apriltag::AprilTagNode',
        remappings=[('/camera/image_rect', '/image_rect'),
                    ('/camera/camera_info', '/camera_info')],
        parameters=[cfg_36h11])

    apriltag_container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    argus_node = Node(
            name='argus',
            package='isaac_ros_argus_camera_mono',
            executable='isaac_ros_argus_camera_mono',
            remappings=[('/image_raw', '/image')],
            parameters=[{'device':0,'sensor':4,'output_encoding':'mono8','camera_info_url':cat_path}]

    )

    return launch.LaunchDescription([argus_node,rectify_container, apriltag_container])
