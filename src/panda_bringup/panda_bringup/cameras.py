import math
import xml.etree.ElementTree as ET


PROFILES = {
    "D435": {"fovy": 42.5, "min_depth_m": .28, "max_depth_m": 3.0,
             "housing_size_m": [.090, .025, .025], "housing_position_m": [0, 0, -.0125],
             "housing_mass_kg": .072, "lens_x_m": [-.025, .025]},
    "D405": {"fovx": 84., "fovy": 58., "min_depth_m": .10, "max_depth_m": .50,
             "housing_size_m": [.042, .042, .023], "housing_position_m": [.009, 0, -.0115],
             "housing_mass_kg": .060, "lens_x_m": [0, .018]},
}


def numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def add_cameras(world, panda, metadata, enabled=("external", "wrist")):
    import mujoco
    import numpy as np

    metadata["cameras"] = []
    for name in enabled:
        if name == "external":
            position = np.array([.85, -.65, 1.55])
            target = np.array([.23, -.02, 0])
            z = (target-position)/np.linalg.norm(target-position)
            x = np.cross(z, [0, 0, 1])
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            q = np.empty(4)
            mujoco.mju_mat2Quat(q, np.column_stack((x, y, z)).flatten())
            parent, container, prefix, frame = "world", world, "camera", "workspace_camera_optical_frame"
            mj_name = "workspace_camera"
            model_name = "D435"
        else:
            position, q = np.array([0, -.080, .035]), np.array([1., 0, 0, 0])
            parent, container, prefix, frame = "panda_hand", panda.find(".//body[@name='panda_hand']"), "wrist_camera", "wrist_camera_optical_frame"
            mj_name = "wrist_camera"
            model_name = "D405"
        profile = PROFILES[model_name]
        width, height = 1280, 720
        fy = height/2/math.tan(math.radians(profile["fovy"])/2)
        fx = width/2/math.tan(math.radians(profile["fovx"])/2) if "fovx" in profile else fy
        housing_position = profile["housing_position_m"]
        body = ET.SubElement(container, "body", name=frame, pos=numbers(position), quat=numbers(q), gravcomp="1")
        ET.SubElement(body, "geom", name=name+"_camera_housing", type="box",
                      size=numbers([v/2 for v in profile["housing_size_m"]]),
                      pos=numbers(housing_position), rgba=".16 .18 .20 1",
                      mass=str(profile["housing_mass_kg"]), group="0")
        for x in profile["lens_x_m"]:
            ET.SubElement(body, "geom", type="cylinder", size=".004 .0005", pos=numbers([x, 0, .0005]),
                          rgba=".02 .03 .05 1", mass="0", contype="0", conaffinity="0", group="2")
        if name == "wrist":
            ET.SubElement(body, "geom", name="wrist_camera_bracket", type="box", size=".012 .025 .004",
                          pos=numbers([housing_position[0], .032, -.015]), rgba=".3 .32 .34 1", mass=".020", group="0")
        optics = {"fovy": str(profile["fovy"])}
        if model_name == "D405":
            optics = {"focalpixel": numbers([fx, fy]), "sensorsize": "1 1"}
        ET.SubElement(body, "camera", name=mj_name, mode="fixed", resolution=f"{width} {height}",
                      quat="0 1 0 0", **optics)
        metadata["cameras"].append({
            **profile, "name": name, "model": model_name, "mj_name": mj_name, "parent": parent, "prefix": prefix,
            "frame": frame, "position": position.tolist(), "optical_quaternion_wxyz": q.tolist(),
            "width": width, "height": height, "fx": fx, "fy": fy,
        })


def add_camera_links(root, cameras):
    for camera in cameras:
        frame = camera["frame"]
        link = ET.SubElement(root, "link", name=frame)
        joint = ET.SubElement(root, "joint", name=camera["name"]+"_camera_mount", type="fixed")
        ET.SubElement(joint, "parent", link=camera["parent"])
        ET.SubElement(joint, "child", link=frame)
        w, x, y, z = camera["optical_quaternion_wxyz"]
        rpy = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
               math.asin(max(-1., min(1., 2*(w*y-z*x)))),
               math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
        ET.SubElement(joint, "origin", xyz=numbers(camera["position"]), rpy=numbers(rpy))
        boxes = [(camera["housing_position_m"], camera["housing_size_m"])]
        if camera["name"] == "wrist":
            boxes.append(([camera["housing_position_m"][0], .032, -.015], [.024, .050, .008]))
        for position, size in boxes:
            for tag in ("visual", "collision"):
                element = ET.SubElement(link, tag)
                ET.SubElement(element, "origin", xyz=numbers(position))
                ET.SubElement(ET.SubElement(element, "geometry"), "box", size=numbers(size))
                if tag == "visual":
                    ET.SubElement(ET.SubElement(element, "material", name="camera_grey"), "color", rgba=".16 .18 .20 1")
