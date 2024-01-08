import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    #Include robot_state_publisher launch file. Force sim time enabled
    #Ensure you update the package name correctly

    package_name='car_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux", 
        parameters=[twist_mux_params, {'use_sim_time':True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    #Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py' )]),
    )
    
    #Run the spawner node from the gazebo_ros package. Entity only matters with multiple robots
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'car_bot'],
                                   output='screen')
    
    #Run the telop keyboard node
    teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_node',
            output='screen',
            prefix='xterm -e'
         )

    #Launch rsp, gazebo and spawn_entity
    return LaunchDescription([
        rsp,
        gazebo, 
        spawn_entity,
    ])