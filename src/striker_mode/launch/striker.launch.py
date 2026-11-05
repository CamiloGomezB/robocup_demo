import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _striker_node(context, *args, **kwargs):
    share = os.path.join(get_package_share_directory('striker_mode'), 'config')
    params = [os.path.join(share, 'striker.yaml')]
    # YAML extra (p. ej. striker_real_k1.yaml) encima de striker.yaml
    extra = LaunchConfiguration('config').perform(context).strip()
    if extra:
        path = extra if os.path.isabs(extra) else os.path.join(share, extra)
        if not os.path.exists(path):
            raise RuntimeError(f'No existe el archivo de configuracion: {path}')
        params.append(path)
    # Argumentos del launch: SOLO los que se pasaron explicitamente. Los parametros sueltos le ganan a los YAML,
    # asi que si pasaramos valores por defecto pisarian la configuracion del robot real (p. ej. control_source).
    typed = {'kick_mode': str, 'autostart': bool, 'attack_yaw': float, 'debug_sim': bool, 'control_source': str,
             'fallback_image_topic': str, 'fallback_camera_info_topic': str}
    overrides = {'use_sim_time': ParameterValue(LaunchConfiguration('sim'), value_type=bool)}
    for name, kind in typed.items():
        if LaunchConfiguration(name).perform(context).strip():
            overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=kind)
    params.append(overrides)
    return [Node(package='striker_mode', executable='striker_node', name='striker_node',
                 output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        # Vacio = no se pasa: vale lo que diga el YAML (striker.yaml / config)
        DeclareLaunchArgument('kick_mode', default_value=''),
        DeclareLaunchArgument('autostart', default_value=''),
        DeclareLaunchArgument('attack_yaw', default_value=''),
        DeclareLaunchArgument('debug_sim', default_value=''),
        # topic: arranca con /booster_agent/soccer_game_control (sim) | remote: solo con el control (LB+RB+Y)
        DeclareLaunchArgument('control_source', default_value=''),
        # YAML extra que se carga encima de striker.yaml (nombre dentro de config/ o ruta absoluta)
        DeclareLaunchArgument('config', default_value=''),
        # Detector de respaldo de la pelota: vacio = desactivado (solo YOLO)
        DeclareLaunchArgument('fallback_image_topic', default_value=''),
        DeclareLaunchArgument('fallback_camera_info_topic', default_value=''),
        OpaqueFunction(function=_striker_node),
    ])
