from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='liat_demo_robot',  # Nombre del paquete que contiene el nodo de la cámara RealSense
            executable='camera.py',  # Nombre del script que contiene el nodo de la cámara RealSense
            name='camera',  # Nombre del nodo
            output='screen'
        ),
    ])
