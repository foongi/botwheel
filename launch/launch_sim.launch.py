import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


os.system("unset XDG_SESSION_TYPE")


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='botwheel' #<--- CHANGE ME
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]),
             launch_arguments={'gz_args': '-r ~/gz_worlds/obstacles.sdf'}.items(), 
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=[{'topic': 'robot_description',
                                   'name': 'botwheel',
                                   'sdf_filename': '~/SLAM/src/botwheel/description/model.urdf'}],
                        output='screen')
    
    # RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'diff_drive.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])