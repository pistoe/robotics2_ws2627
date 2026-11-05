from glob import glob
from setuptools import setup

setup(
    name="panda_bringup",
    version="0.1.0",
    packages=["panda_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/panda_bringup"]),
        ("share/panda_bringup", ["package.xml"]),
        ("share/panda_bringup/launch", glob("launch/*.launch.py")),
        ("share/panda_bringup/config", glob("config/*")),
    ],
    entry_points={"console_scripts": ["joint_goal = panda_bringup.joint_goal:main",
                                      "competition_scene = panda_bringup.competition_scene:main",
                                      "camera_bridge = panda_bringup.camera_bridge:main",
                                      "simulation_runtime = panda_bringup.simulation_runtime:main"]},
)
