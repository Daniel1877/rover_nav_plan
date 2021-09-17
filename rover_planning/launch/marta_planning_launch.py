# Copyright 2019 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    planning_dir = get_package_share_directory('rover_planning')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            #'plansys2_bringup_launch_distributed.py')),
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': planning_dir + '/pddl/rover_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          planning_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'bt_xml_file': planning_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    recharge_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='recharge',
        namespace=namespace,
        output='screen',
        parameters=[
          planning_dir + '/config/params.yaml',
          {
            'action_name': 'recharge',
            'bt_xml_file': planning_dir + '/behavior_trees_xml/recharge.xml'
          }
        ])

    photo_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='photo',
        namespace=namespace,
        output='screen',
        parameters=[
          planning_dir + '/config/params.yaml',
          {
            'action_name': 'photo',
            'bt_xml_file': planning_dir + '/behavior_trees_xml/photo.xml'
          }
        ])

    record_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='record',
        namespace=namespace,
        output='screen',
        parameters=[
          planning_dir + '/config/params.yaml',
          {
            'action_name': 'record',
            'bt_xml_file': planning_dir + '/behavior_trees_xml/record.xml'
          }
        ])

    calibration_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='calibration',
        namespace=namespace,
        output='screen',
        parameters=[
          planning_dir + '/config/params.yaml',
          {
            'action_name': 'calibration',
            'bt_xml_file': planning_dir + '/behavior_trees_xml/calibration.xml'
          }
        ])

    '''photo_cmd = Node(
        package='rover_planning',
        executable='photo',
        name='photo',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate'''


    nav_cam_image_saver = Node (
        package = 'image_view',
        executable = 'image_saver',
        name = 'nav_cam_image_saver',
        namespace='nav_cam_depth',
        output='screen',
        parameters=[{
                     'save_all_image': False,
                     'filename_format': "nav_cam%04i.%s"
                     }],
        remappings= [
                      ('/nav_cam_depth/image', '/nav_cam_depth/image_raw'),
                      ('/nav_cam_depth/camera_info', '/nav_cam_depth/camera_info')
                    ]
    )

    loc_cam_image_saver = Node (
        package = 'image_view',
        executable = 'image_saver',
        name = 'loc_cam_image_saver',
        namespace='loc_cam_depth',
        output='screen',
        parameters=[{
                     'save_all_image': False,
                     'filename_format': "loc_cam%04i.%s"
                     }],
        remappings=[('/loc_cam_depth/image', '/loc_cam_depth/image_raw'),
                    ('/loc_cam_depth/camera_info', '/loc_cam_depth/camera_info')
                    ]
    )

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(recharge_cmd)
    ld.add_action(photo_cmd)
    ld.add_action(record_cmd)
    ld.add_action(calibration_cmd)

    ld.add_action(nav_cam_image_saver)
    ld.add_action(loc_cam_image_saver)

    return ld
