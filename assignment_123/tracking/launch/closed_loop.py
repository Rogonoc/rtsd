from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Jiwy simulator
        Node
        (
            package='jiwy_simulator',
            executable='jiwy_simulator',
            name='jiwy',
        ),
        
        # Centre of Gravity
        Node
        (
            package='template',
            executable='template',
            name='cog',
            remappings=[
                ('/webcam_input', '/webcam_output'),
            ]
        ),

        # Cam2image
        Node
        (
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            remappings=[
                ('/image', '/webcam_input'),
            ]
        ),

        # Showimage (jiwy)
        Node
        (
            package='image_tools',
            executable='showimage',
            name='showimage',
            remappings=[
                ('/image', '/webcam_output'),
            ]
        ),

        # Showimage (webcam)
        Node
        (
            package='image_tools',
            executable='showimage',
            name='showimage',
            remappings=[
                ('/image', '/webcam_input'),
            ]
        )

    ])