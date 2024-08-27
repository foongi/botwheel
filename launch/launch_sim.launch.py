import os
import xacro

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node




def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = get_package_share_directory('botwheel')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Parse robot description from xacro
    robot_description_file = os.path.join(package_name, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )

    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    # Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )


    gazebo_params_path = os.path.join(
                  package_name,'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': '-r /home/ethanfoong/gz_worlds/obstacles.sdf'
                                      #'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path
                                      }.items(), 
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        parameters=[{'topic': 'robot_description',
                     'name': 'botwheel'}],
        output='screen')
    
    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(package_name, 'rviz', 'botwheel.rviz')],
    )

    # Bridge to forward tf and joint states to ros2
    gz_topic = '/model/botwheel'
    joint_state_gz_topic = '/world/obstacles' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'q`os`_overrides./tf_static.publisher.durability': 'transient_local',
                     'qos_overrides./model/botwheel.subscriber.reliability': 'reliable',}],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )
    # Launch them all!
    return LaunchDescription([
        
        rviz,
        gazebo,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_controller],
            )
        ),
        
        rsp,
        bridge,
        spawn_entity,
        

        
    ])