import json
import math
from pathlib import Path
import secrets
import tempfile
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from panda_bringup.model import robot_description, settings, share

def _setup(context, backend, competition=False, randomized=False, student=False):
    value = lambda key: LaunchConfiguration(key).perform(context)
    namespace = value('namespace').strip('/')
    headless = value('headless').lower() == 'true'
    activate_controller = value('activate_controller').lower() == 'true'
    use_moveit = value('use_moveit').lower() == 'true' and activate_controller
    controllers = share() / 'config/controllers.yaml'
    robot_ip = ''
    scene, environment = ('', None)
    simulation_options = {}
    cameras = []
    if competition:
        from panda_bringup.competition import build_scene
        output = Path('/data/competition_randomized' if randomized else '/data/competition') if Path('/data').is_dir() else Path(tempfile.mkdtemp(prefix='panda-competition-'))
        seed = secrets.randbits(32) if randomized and value('seed') == 'random' else int(value('seed'))
        enabled_cameras = tuple((name for name in ('external', 'wrist') if value('camera').lower() == 'true' and value(name + '_camera').lower() == 'true'))
        camera_rate = float(value('camera_rate'))
        if not math.isfinite(camera_rate) or not 0 < camera_rate <= 30:
            raise ValueError('camera_rate must be finite and in (0, 30] Hz')
        scene, environment = build_scene(share('panda_description'), value('stage'), seed, output, value('layout') or None, randomized=randomized, cameras=enabled_cameras)
        plugin_config = share() / 'config/competition_plugins.yaml'
        plugins = yaml.safe_load(plugin_config.read_text())
        entries = plugins['/**']['ros__parameters']['mujoco_plugins']
        print(f'Continuous competition: seed={seed}, randomized={randomized}', flush=True)
        cameras = json.loads(environment.read_text()).get('cameras', [])
        public = json.loads(environment.read_text())
        public = {key: public[key] for key in ('stage', 'geometry', 'cameras', 'instances')}
        public['instances'] = [item for item in public['instances'] if not item['dynamic']]
        public_path = output / 'public_scene.json'
        public_path.write_text(json.dumps(public) + '\n')
        environment = public_path
        if cameras:
            entries['rgbd_cameras'] = {'type': 'mujoco_ros2_control_plugins/CameraPlugin', 'camera_publish_rate': camera_rate, **{camera['mj_name']: {'frame_name': camera['frame'], 'info_topic': f"/{namespace}/{camera['prefix']}/_native_camera_info", 'image_topic': f"/{namespace}/{camera['prefix']}/color/image_raw", 'depth_topic': f"/{namespace}/{camera['prefix']}/_native_depth"} for camera in cameras}}
        plugin_config = output / 'plugins.yaml'
        plugin_config.write_text(yaml.safe_dump(plugins))
        simulation_options = {'simulation_config': str(plugin_config), 'initial_keyframe': 'competition_home', 'floor_height': 0.0, 'cameras': cameras}
    else:
        scene = value('scene')
    description = {'robot_description': robot_description('simulation', headless, robot_ip, str(scene), **simulation_options)}
    clock = {'use_sim_time': True}
    semantics = {'robot_description_semantic': (share() / 'config/panda.srdf').read_text()}
    if any((camera['name'] == 'wrist' for camera in cameras)):
        semantic_root = ET.fromstring(semantics['robot_description_semantic'])
        ET.SubElement(semantic_root, 'disable_collisions', link1='panda_hand', link2='wrist_camera_optical_frame', reason='Adjacent')
        semantics['robot_description_semantic'] = ET.tostring(semantic_root, encoding='unicode')
    planning = yaml.safe_load((share() / 'config/moveit.yaml').read_text())
    robot = settings()
    planning['robot_description_planning'] = {'default_velocity_scaling_factor': 0.1, 'default_acceleration_scaling_factor': 0.1, 'joint_limits': {name: {'has_velocity_limits': True, 'max_velocity': robot['velocity'][i], 'has_acceleration_limits': True, 'max_acceleration': 1.0} for i, name in enumerate(robot['joint_names'])}}
    planning['robot_description_planning']['joint_limits']['panda_finger_joint1'] = {'has_velocity_limits': True, 'max_velocity': 0.05, 'has_acceleration_limits': True, 'max_acceleration': 0.25}
    manager = Node(package='mujoco_ros2_control', executable='ros2_control_node', namespace=namespace, parameters=[str(controllers), clock], output='screen', on_exit=[EmitEvent(event=Shutdown(reason='Controller manager stopped'))])
    spawner = Node(package='controller_manager', executable='spawner', namespace=namespace, arguments=['joint_state_broadcaster', *(['panda_arm_controller'] if activate_controller else []), '--activate-as-group', '--controller-manager', f'/{namespace}/controller_manager', '--controller-manager-timeout', '60', '--switch-timeout', '30', '--param-file', str(controllers)], output='screen')
    nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace, parameters=[description, clock, {'publish_frequency': 100.0, 'ignore_timestamp': True}], output='screen'), manager, spawner]
    nodes.append(Node(package='panda_bringup', executable='simulation_runtime', namespace=namespace, parameters=[clock, {'arm_enabled': activate_controller, 'resume_on_start': value('start_paused').lower() != 'true'}], output='screen', on_exit=[EmitEvent(event=Shutdown(reason='Simulation runtime coordinator stopped'))]))
    if competition:
        nodes.append(Node(package='panda_bringup', executable='competition_scene', namespace=namespace, parameters=[{'scene_description': str(environment), 'allow_fixture_support_contacts': True}, clock], output='screen', on_exit=[EmitEvent(event=Shutdown(reason='Competition scene bridge stopped'))]))
    if cameras:
        nodes.append(Node(package='panda_bringup', executable='camera_bridge', namespace=namespace, parameters=[{'scene_description': str(environment)}, clock], output='screen', on_exit=[EmitEvent(event=Shutdown(reason='Camera bridge stopped'))]))
    if use_moveit:
        move_group = Node(package='moveit_ros_move_group', executable='move_group', namespace=namespace, parameters=[description, semantics, planning, clock], output='screen', additional_env={'LD_PRELOAD': 'libmoveit_simple_controller_manager.so'}, on_exit=[EmitEvent(event=Shutdown(reason='MoveIt stopped'))])

    def after_controllers(event, context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Controller activation failed'))]
        return [move_group] if use_moveit else []
    nodes.insert(0, RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_controllers)))
    if value('use_rviz').lower() == 'true':
        rviz_config = yaml.safe_load((share() / 'config/panda.rviz').read_text())
        displays = rviz_config['Visualization Manager']['Displays']
        for display in displays:
            if display['Class'] == 'rviz_default_plugins/RobotModel':
                display['Description Topic']['Value'] = f'/{namespace}/robot_description'
            if display['Class'] == 'moveit_rviz_plugin/MotionPlanning':
                display['Planning Scene Topic'] = f'/{namespace}/monitored_planning_scene'
                display['Move Group Namespace'] = f'/{namespace}'
                if competition:
                    display['Scene Geometry'] = {'Show Scene Geometry': not False}
        if competition:
            if False or not use_moveit:
                displays.append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Competition', 'Enabled': True, 'Topic': {'Value': f'/{namespace}/competition/markers'}})
            for camera in cameras:
                displays.append({'Class': 'panda_rviz/Image', 'Name': camera['name'].title() + ' ' + camera['model'], 'Enabled': True, 'Topic': {'Value': f"/{namespace}/{camera['prefix']}/color/image_raw"}})
            rviz_config['Visualization Manager']['Views']['Current']['Focal Point'] = {'X': 0.3, 'Y': -0.05, 'Z': 0.15}
            rviz_config['Window Geometry'] = {'Width': 1800, 'Height': 1000}
        if not use_moveit:
            rviz_config['Visualization Manager']['Displays'] = [display for display in displays if display['Class'] != 'moveit_rviz_plugin/MotionPlanning']
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False) as config_file:
            yaml.safe_dump(rviz_config, config_file)
        nodes.append(Node(package='rviz2', executable='rviz2', namespace=namespace, arguments=['-d', config_file.name], additional_env={'LD_PRELOAD': 'librmw_cyclonedds_cpp.so'}, parameters=[description, semantics, {'robot_description_kinematics': planning['robot_description_kinematics']}, clock], output='screen'))
    return nodes

def generate(backend, competition=False, randomized=False, student=False):
    arguments = [DeclareLaunchArgument('namespace', default_value='panda'), DeclareLaunchArgument('use_rviz', default_value='true'), DeclareLaunchArgument('use_moveit', default_value='true'), DeclareLaunchArgument('activate_controller', default_value='true')]
    arguments.append(DeclareLaunchArgument('headless', default_value='false'))
    arguments.append(DeclareLaunchArgument('start_paused', default_value='false', description='Keep physics paused after controller readiness'))
    if competition:
        arguments.extend([DeclareLaunchArgument('stage', default_value='continuous', choices=['continuous', 'all'], description='One initial state for all six levels; all is a compatibility alias'), DeclareLaunchArgument('seed', default_value='random' if randomized else '0'), DeclareLaunchArgument('layout', default_value='', description='Optional competition layout JSON (metres)'), DeclareLaunchArgument('camera', default_value='true', description='Enable competition camera rig'), DeclareLaunchArgument('external_camera', default_value='true'), DeclareLaunchArgument('wrist_camera', default_value='true'), DeclareLaunchArgument('camera_rate', default_value='5.0', description='RGB-D publication rate per camera (max 30 Hz)')])
    else:
        arguments.append(DeclareLaunchArgument('scene', default_value='', description='Optional MuJoCo scene path'))
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup, kwargs={'backend': 'simulation', 'competition': competition, 'randomized': randomized, 'student': True})])
