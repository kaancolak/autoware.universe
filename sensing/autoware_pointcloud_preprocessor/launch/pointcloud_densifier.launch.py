# Copyright 2025 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch the pointcloud densifier node."""
    pkg_prefix = get_package_share_directory('autoware_pointcloud_preprocessor')
    config_file = os.path.join(pkg_prefix, 'config/pointcloud_densifier.param.yaml')

    # Declare only the arguments that might be overridden at launch time
    point_type = DeclareLaunchArgument(
        "point_type",
        default_value="autoware::point_types::PointXYZIRC",
        description="Point type to use (pcl::PointXYZ, pcl::PointXYZI, autoware::point_types::PointXYZIRC)",
    )
    
    input_topic = DeclareLaunchArgument(
        "input_topic",
        default_value="/sensing/lidar/concatenated/pointcloud",
        description="Input pointcloud topic",
    )
    
    output_topic = DeclareLaunchArgument(
        "output_topic",
        default_value="/sensing/lidar/densified/pointcloud",
        description="Output pointcloud topic",
    )

    # Create our custom launcher node
    densifier_node = Node(
        package="autoware_pointcloud_preprocessor",
        executable="pointcloud_densifier_launcher",
        name="pointcloud_densifier",
        remappings=[
            ("input", LaunchConfiguration("input_topic")),
            ("output", LaunchConfiguration("output_topic")),
        ],
        parameters=[
            config_file,  # Load parameters from config file first
            {
                "point_type": LaunchConfiguration("point_type"),  # Override point_type from launch args
            }
        ],
    )

    return launch.LaunchDescription([
        # Launch arguments - only those that might be overridden
        point_type,
        input_topic,
        output_topic,
        # Nodes
        densifier_node,
    ])