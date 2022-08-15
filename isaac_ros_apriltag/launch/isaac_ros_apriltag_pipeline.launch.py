# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::RectifyNode',
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

    apriltag_node = ComposableNode(
      package='isaac_ros_apriltag',
      plugin='isaac_ros::apriltag::AprilTagNode',
      name='apriltag',
      parameters=[{'size': 0.16,
                   'max_tags': 64}])

    apriltag_container = ComposableNodeContainer(
      package='rclcpp_components',
      name='apriltag_container',
      namespace='',
      executable='component_container_mt',
      composable_node_descriptions=[
          apriltag_node,
      ],
      output='screen'
    )

    return launch.LaunchDescription([rectify_container, apriltag_container])
