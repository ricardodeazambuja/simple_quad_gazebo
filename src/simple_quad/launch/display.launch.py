import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from os.path import exists
import re
import random

# from scripts import GazeboRosPaths


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='simple_quad').find('simple_quad')

    #
    # Path fix and texture randomization...
    # 
    # Using this hack because I got tired of the problems trying to use anything but file://
    gazebo_world_path = os.path.join(pkg_share, 'world')
    with open(os.path.join(gazebo_world_path, 'my_world.sdf'), 'r') as file:
        filedata = file.read()
    filedata = filedata.replace('LAUNCH_FILE_REPLACE', gazebo_world_path)
    material_list = [f'texture_{i+1:03d}' for i in range(10)]
    for i in range(filedata.count('TEXTURE')):
        filedata = filedata.replace('TEXTURE', random.choice(material_list), 2)
    with open(os.path.join(gazebo_world_path, 'my_world_path_corrected.sdf'), 'w') as file:
        file.write(filedata)

    # model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    # env = {
    #     "GAZEBO_MODEL_PATH": model_path,
    #     # "GAZEBO_PLUGIN_PATH": plugin_path,
    #     # "GAZEBO_RESOURCE_PATH": media_path,
    # }

    
    default_model_path = os.path.join(pkg_share, 'src/description/simple_quad.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher' 
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_quad', '-topic', 'robot_description', '-z', '0.05'],
        output='screen'
    )

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        remappings=[('/cmd_vel', '/simple_quad/cmd_vel_rl')],
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    world_path=os.path.join(pkg_share, 'world/my_world_path_corrected.sdf'),

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
                                                                  '-s', 'libgazebo_ros_factory.so',
                                                                  world_path], 
                                      output='screen'),
                                    #   additional_env=env), # env=env will REPLACE all the variables instead of appending
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node
    ])
