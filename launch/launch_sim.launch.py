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

    #Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch',
            'gazebo.launch.py' )]),
    )
    
    #Run the spawner node from the gazebo_ros package. Entity only matters with multiple robots
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                                   output='screen')
    
    #Launch rsp, gazebo and spawn_entity
    return LaunchDescription([
        rsp,
        gazebo, 
        spawn_entity,
    ])