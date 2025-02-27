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

    share_dir = '/home/pi/adc_ws/src/parking/config'    #get_package_share_directory('rc_receiver_package')
    config_cross_in_path = os.path.join(
        os.getcwd(),
        'src',
        'parking',
        'config',
        'params_cross_in.yaml'
        )
    
    config_cross_in_path = os.path.join(
        os.getcwd(),
        'src',
        'parking',
        'config',
        'params_parallel_in.yaml'
        )

    node_name = 'parking_node'
    params_declare = DeclareLaunchArgument(
        'params-file',
        default_value=os.path.join(
            share_dir, 'parking_params.yaml'),
            description='File path to the ROS2 parameters file to use'
    )
    params_cross_in_declare = DeclareLaunchArgument(
        'cross_in_params-file',
        default_value = 
            config_cross_in_path,
            description = 'File path to the ROS2 parameters file to use'
    )

    params_parallel_in_declare = DeclareLaunchArgument(
        'parallel_in_params-file',
        default_value = 
            config_cross_in_path,
            description = 'File path to the ROS2 parameters file to use'
    )

    ultrasound_node_dir = get_package_share_directory('ultrasound')
    included_launch_ultrasound = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                ultrasound_node_dir + '/launch/ultrasound_node_launch.py'))     
	
    parking_nav_node = Node(
        package='parking',
        executable='parking_nav_pixel',
        name=node_name,
        #namespace=TextSubstitution(text=''),
        #namespace='car1',
        output='screen',
    )
    parking_node = Node(
        package='parking',
        executable='parking_node',
        name=node_name,
        #namespace=TextSubstitution(text=''),
        #namespace='car1',
        parameters=[LaunchConfiguration('params-file'),
                    LaunchConfiguration('parallel_in_params-file'),
                    LaunchConfiguration('cross_in_params-file')],
        output='screen',
    )
    return LaunchDescription([
    # launch_include,
        params_declare,
        params_cross_in_declare,
        params_parallel_in_declare,
        #included_launch_ultrasound,
        #parking_nav_node,
        parking_node,
    	])
