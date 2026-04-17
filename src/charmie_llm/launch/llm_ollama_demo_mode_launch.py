from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='charmie_llm',
            executable='llm_ollama',
            name='llm_ollama',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'task': 'demo',
            }]
        )
    ])