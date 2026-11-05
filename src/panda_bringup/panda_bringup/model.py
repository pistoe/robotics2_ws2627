from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory


def share(package="panda_bringup"):
    return Path(get_package_share_directory(package))


def settings():
    return yaml.safe_load((share() / "config/robot.yaml").read_text())


def robot_description(backend, headless=False, robot_ip="", scene="", simulation_config="",
                      initial_keyframe="home", floor_height=-0.01, cameras=()):
    if backend not in {"simulation", "hardware"}:
        raise ValueError(f"Unknown backend: {backend}")
    config = settings()
    root = ET.parse(share("moveit_resources_panda_description") / "urdf/panda.urdf").getroot()
    for name in ("panda_finger_joint1", "panda_finger_joint2"):
        limit = root.find(f"joint[@name='{name}']/limit")
        limit.set("effort", "70")
        limit.set("velocity", "0.05")
    ET.SubElement(root, "link", name="world")
    mount = ET.SubElement(root, "joint", name="panda_mount", type="fixed")
    ET.SubElement(mount, "parent", link="world")
    ET.SubElement(mount, "child", link="panda_link0")
    floor = ET.SubElement(root, "link", name="panda_floor")
    geometry = ET.SubElement(ET.SubElement(floor, "collision"), "geometry")
    ET.SubElement(geometry, "box", size="4 4 0.02")
    floor_joint = ET.SubElement(root, "joint", name="panda_floor_joint", type="fixed")
    ET.SubElement(floor_joint, "parent", link="world")
    ET.SubElement(floor_joint, "child", link="panda_floor")
    ET.SubElement(floor_joint, "origin", xyz=f"0 0 {floor_height-0.01}", rpy="0 0 0")
    if root.find("link[@name='panda_hand_tcp']") is None:
        ET.SubElement(root, "link", name="panda_hand_tcp")
        tcp = ET.SubElement(root, "joint", name="panda_tcp_joint", type="fixed")
        ET.SubElement(tcp, "parent", link="panda_hand")
        ET.SubElement(tcp, "child", link="panda_hand_tcp")
        ET.SubElement(tcp, "origin", xyz="0 0 0.1034", rpy="0 0 0")
    if cameras:
        from panda_bringup.cameras import add_camera_links
        add_camera_links(root, cameras)
    control = ET.SubElement(root, "ros2_control", name="PandaSystem", type="system")
    hardware = ET.SubElement(control, "hardware")
    plugin = ET.SubElement(hardware, "plugin")
    if backend == "simulation":
        plugin.text = "mujoco_ros2_control/MujocoSystemInterface"
        params = {
            "mujoco_model": str(Path(scene).expanduser().resolve(strict=True)) if scene else
                            str(share("panda_description") / "mjcf/scene.xml"),
            "headless": str(headless).lower(),
            "sim_speed_factor": "1.0",
            "initial_keyframe": initial_keyframe,
            "start_paused": "true",
            "pids_config_file": simulation_config or str(share() / "config/gripper.yaml"),
        }
    else:
        if not robot_ip:
            raise ValueError("Hardware requires robot_ip")
        plugin.text = "franka_hardware/FrankaMultiHardwareInterface"
        params = {"robot_count": "1", "ns_1": "panda", "robot_ip_1": robot_ip}
    for key, value in params.items():
        ET.SubElement(hardware, "param", name=key).text = value
    for index, name in enumerate(config["joint_names"]):
        joint = root.find(f"joint[@name='{name}']")
        limit = joint.find("limit")
        for key in ("lower", "upper", "effort", "velocity"):
            limit.set(key, str(config[key][index]))
        safety = joint.find("safety_controller")
        if safety is not None:
            safety.set("soft_lower_limit", str(config["lower"][index]))
            safety.set("soft_upper_limit", str(config["upper"][index]))
        interface = ET.SubElement(control, "joint", name=name)
        commands = ["effort"] if backend == "simulation" else ["effort", "position", "velocity"]
        for command in commands:
            command_tag = ET.SubElement(interface, "command_interface", name=command)
            if command == "effort":
                ET.SubElement(command_tag, "param", name="min").text = str(-config["effort"][index])
                ET.SubElement(command_tag, "param", name="max").text = str(config["effort"][index])
        for state in ("position", "velocity", "effort"):
            state_tag = ET.SubElement(interface, "state_interface", name=state)
            if state == "position" and backend == "simulation":
                ET.SubElement(state_tag, "param", name="initial_value").text = str(config["home"][index])
    return ET.tostring(root, encoding="unicode")
