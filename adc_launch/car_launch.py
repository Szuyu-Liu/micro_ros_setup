from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_ros.descriptions
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os

def generate_launch_description():

    launchdescription = LaunchDescription()

    teensy_serial_short = '14118480' #14089910  14089710

    rc_node_dir = get_package_share_directory('rc_receiver')
    included_launch_rc = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                rc_node_dir + '/launch/rc_receiver_launch.py'))

    ultrasound_node_dir = get_package_share_directory('ultrasound')
    included_launch_ultrasound = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                ultrasound_node_dir + '/launch/ultrasound_node_launch.py'))        

    parking_node_dir = get_package_share_directory('parking')
    included_launch_parking = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                parking_node_dir + '/launch/parking_launch.py'))    

    ydlidar_node_dir = get_package_share_directory('ydlidar_ros2_driver')
    included_launch_ydlidar = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                ydlidar_node_dir + '/launch/ydlidar_launch.py'))       

    camera_navigation_node_pixel_based = Node(
        package='navigation_pixel-based',
        executable='navigation_pixel-based',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='navigation_pixel_based_node',
        output='screen'
    )

    camera_navigation_node_hough_transform = Node(
        package='navigation_hough-transform',
        executable='navigation_hough-transform',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='navigation_hough_transform_node',
        output='screen'
    )

    neural_network_node = Node(
        package='neural_networks',
        executable='neural_networks',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='NN_Drive',
        output='screen'
    )

    neural_network_node_cpp = Node(
        package='neural_networks_cpp',
        executable='neural_networks_cpp',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='NNDrive',
        output='screen'
    )
    
    oak_adc_setup = Node(
        package='oak_camera',
        executable='adc_camera_setup',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='stereo_node',
        output='screen'
    )

    # Launch gui_control node
    gui_control_node = Node(
        package='gui_control',
        executable='gui_control',
        namespace=TextSubstitution(text=''),
        # namespace='car1',
        name='gui_control_node',
        output='screen'
    )
    # Launch micro_ros_agent node
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='micro_ros_agent_node',
        output='screen',
        arguments=['serial', '--dev', teensy_serial_short]
    )
    # Launch drive_controller node
    drive_controller_node = Node(
        package='drive_controller',
        executable='drive_controller',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='drive_controller_node',
        output='screen'
    )
    #Launch Lidar
    lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )
    
    # Launch IMU
    imu_node = Node(
        package='imu_bno055',
        executable='imu_node',
        namespace=TextSubstitution(text=''),
        #namespace='car1',
        name='imu_node',
        output='screen'
    )
    # Launch Overtaking
    overtaking_node = Node(
       package='overtaking',
       executable='overtaking',
       namespace=TextSubstitution(text=''),
       #namespace='car1',
       name='overtaking_node',
       output='screen'
    )

    launchdescription.add_action(included_launch_rc)
    launchdescription.add_action(included_launch_ultrasound)
    #launchdescription.add_action(included_launch_parking)  no launchfile in src/parking/launch
    launchdescription.add_action(gui_control_node)
    launchdescription.add_action(micro_ros_agent_node)
    launchdescription.add_action(imu_node)
    launchdescription.add_action(drive_controller_node)
    #launchdescription.add_action(oak_adc_setup)
    #launchdescription.add_action(overtaking_node)
    #launchdescription.add_action(lidar_node)
    #launchdescription.add_action(neural_network_node)
    #launchdescription.add_action(camera_navigation_node_pixel_based)
    #launchdescription.add_action(camera_navigation_node_hough_transform)
    launchdescription.add_action(included_launch_ydlidar)
    

    return launchdescription
    
    
