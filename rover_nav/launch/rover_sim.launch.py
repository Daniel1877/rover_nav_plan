import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_helpers import to_urdf

from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    config_dir = get_package_share_directory('rover_config')
    navigation_dir = get_package_share_directory('rover_nav')
    worlds_dir = os.path.join(config_dir, 'worlds')
    #pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    #rviz_config= os.path.join(config_dir, 'rviz', 'marta.rviz')
    rviz_config= os.path.join(config_dir, 'rviz', 'nav2_default_view.rviz')
        #rviz_config=os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'),

    xacro_model = os.path.join(config_dir, 'urdf', 'marta.xacro')
    urdf_model_path, robot_desc = to_urdf(xacro_model)

    #urdf_model = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_simulator = LaunchConfiguration('use_simulator')

    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        #default_value=os.path.join(config_dir, 'worlds', 'tb3_world.model'),
        default_value=os.path.join(config_dir, 'worlds', 'mars_walls.world'),
        #default_value=os.path.join(config_dir, 'worlds', 'base.world'),
        description='Full path to world model file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='Whether to start the simulator.')

    declare_use_gazebo_gui_cmd = DeclareLaunchArgument(
        'use_gazebo_gui',
        default_value='true',
        description='Whether to execute the gui (gzclient).')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        #default_value=os.path.join(config_dir, 'maps', 'map.yaml'),
        default_value=os.path.join(config_dir, 'maps', 'mars.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(navigation_dir, 'params', 'navigation_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('rover_nav'),
            'behaviour_trees', 'navigate.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    remappings = [('/tf', 'tf'),
              ('/tf_static', 'tf_static')]


    """start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gazebo_gui,
                          'server': use_simulator}.items() )
                          #'world': world}.items() )
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so' ],
        output='screen')"""

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'gui': use_gazebo_gui,
                          'server': use_simulator,
                          'world': world}.items() )

    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace='rover',
        emulate_tty=True,
        arguments=['-entity',
                   'marta',
                   '-x', '-1.5', '-y', '0.5', '-z', '1',
                   #'-x', '-6.0', '-y', '0.0', '-z', '1',
                   #'-x', '0.0', '-y', '0.0', '-z', '1',
                   '-file', urdf_model_path,
                   #'-file', urdf_model,
                   '-reference_frame', 'world']
    )

    start_robot_state_publisher_cmd = Node(
        #condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        #namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        #parameters=[configured_params],
        remappings=[('/joint_states', '/joint_states')],
        arguments=[urdf_model_path]
    )

    """start_robot_state_publisher_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', urdf_model_path],
        output='screen')"""

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        # Use this option to see all output from rviz:
        # output='screen',
        # Use this option to supress messages of missing tfs,
        # at startup of rviz and gazebo:
        output={'stdout':'log'},
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings
    )

    # Static tf from
    localization_cam_link = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 name='localization_camera_link',
                                 arguments = ["0.3", "0", "0.45", "1.58", "3.14", "0.74", "base_footprint", "localization_cam"], #x y z yaw pitch roll
                                 output='screen'
                                 )

    navigation_cam_link = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 name='localization_camera_link',
                                 arguments = ["0.187", "0", "0.83", "1.58", "3.14", "1.57", "base_footprint", "navigation_cam"], #x y z yaw pitch roll
                                 output='screen'
                                 )
#The joint state publisher is a ROS2 package which publishes the state for the non-fixed joints.
    """joint_state_pub_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_node',
        output='screen',
        parameters=[configured_params]
    )"""

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_dir, "launch", 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_gazebo_gui_cmd)
    ld.add_action(declare_use_simulator_cmd)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(start_gazebo)
    ld.add_action(spawn_rover)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(localization_cam_link)
    ld.add_action(navigation_cam_link)

    ld.add_action(bringup_cmd)

    #ld.add_action(joint_state_pub_cmd)
    return ld
