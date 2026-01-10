from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

UPDATE_MODELS = 10

def generate_launch_description():

    # Resolve the full path to the RViz configuration file
    rviz_config_file = os.path.join(get_package_share_directory('rviz_vis_model'), 
                                    'models', 
                                    'vis_eye_scalpel.rviz'
                                    )

        
    return LaunchDescription([

        # launch rviz with safed settings 
        Node(
            package= 'rviz2',
            executable= 'rviz2',
            name= 'rviz2',
            arguments=['-d'+ rviz_config_file]
        ),

        # launch the models for the eye staticly
        Node(
            package= 'rviz_vis_model',
            executable= 'eye_model',
            name= 'eye_model',
            output= 'screen',
            parameters=[
                {"updateSeconds":UPDATE_MODELS}
            ]
        ),

        # launch the scalpel model
        Node(
            package= 'rviz_vis_model',
            executable= 'cut_model',
            name= 'cut_model',
            output= 'screen',
        ),


        # static transform from 'scalpel' (3D-Model) into 'map'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='eye_to_world',
            output='screen',
            arguments=['0.0', '0.0', '0.0','0.0', '0.0', '0.0','eye', 'world']
        )

    ])