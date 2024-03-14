from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Obtener la ruta del paquete actual
    package_dir = os.path.dirname(os.path.abspath(__file__))

    # Definir la ruta a los archivos de lanzamiento que deseas invocar
    launcher1 = os.path.join(package_dir, 'liat_demo_hunter-teleop.launch.py')
    launcher2 = os.path.join(package_dir, 'demo_arm.launch.py')

    # Crear una descripción de lanzamiento
    ld = LaunchDescription()

    # Incluir los lanzadores
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launcher1)
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launcher2)
        )
    )

    return ld

