import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


COLORS = {"R": [0.85, 0.12, 0.10, 1], "G": [0.15, 0.65, 0.20, 1],
          "B": [0.12, 0.30, 0.85, 1], "Y": [0.95, 0.75, 0.08, 1],
          "V": [0.55, 0.18, 0.75, 1], "orange": [1, 0.40, 0.05, 1],
          "turquoise": [0.05, 0.70, 0.65, 1], "fixture": [0.46, 0.53, 0.57, 1]}


def numbers(values):
    return " ".join(f"{v:.10g}" for v in values)


def rotate_xy(x, y, yaw):
    return math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y


def placement(config, offset=(0, 0, 0)):
    x, y = rotate_xy(*offset[:2], config["yaw"])
    p = config["position"]
    return [p[0]+x, p[1]+y, p[2]+offset[2]], [math.cos(config["yaw"]/2), 0, 0, math.sin(config["yaw"]/2)]


def object_poses(stage, seed, layout):
    if stage not in ("continuous","all"):
        raise ValueError("Use stage:=continuous: Levels 1-6 share one state; separate stage starts are retired")
    ids=[f"O{kind}_{color}" for kind in (1,2,3) for color in ("R","G","B")]+["O4_Y","O4_V","O5_1","O6_1","O7_1","O8_1"]
    types={"O1":"O1_cube","O2":"O2_cuboid","O3":"O3_cylinder","O4":"O4_l_piece",
           "O5":"O5_round_peg","O6":"O6_d_peg","O7":"O7_product_base","O8":"O8_product_cover"}
    special={"O5":COLORS["orange"],"O6":COLORS["turquoise"],"O7":[.72,.75,.78,1],"O8":[.24,.30,.36,1]}
    objects=[]
    for name in ids:
        kind=name[:2]
        objects.append({"name":"competition_"+name,"parts":[{"type":types[kind],"offset":[0,0,0]}],
                        "position":[0,0,0],"quaternion":[1,0,0,0],
                        "color":COLORS[name[-1]] if kind in ("O1","O2","O3","O4") else special[kind],"dynamic":True})
    from panda_bringup.randomization import randomize
    return randomize(objects,stage,seed,layout)


def decorations(instance, stage):
    result = []
    def paint(shape, size, position, quaternion=(1, 0, 0, 0), color=(1, 1, 1, 1), label=""):
        result.append({"shape": shape, "size": list(size), "position": list(position),
                       "quaternion": list(quaternion), "color": list(color), "label": label})
    for part in instance["parts"]:
        kind = part["type"]
        if kind == "O2_cuboid":
            paint("cylinder", (.002, .00005), (.025, .015, .04005))
            paint("box", (.004, .00005, .008), (.025, -.00005, .020), color=(0, 0, 0, 1))
        elif kind == "O4_l_piece":
            paint("cylinder", (.002, .00005), (.011, .011, .03005))
        elif kind == "O6_d_peg":
            paint("box", (.00005, .003, .008), (.03205, .016, .015))
        elif kind == "C1_l_target":
            x, y, z = part["offset"]
            paint("cylinder", (.002, .00005), (x+.070, y+.074, z+.00805))
        elif kind == "C4_product_nest":
            x,y,z=part["offset"]
            paint("box",(.008,.001,.00005),(x+.075,y+.082,z+.00805))
        elif kind == "B1_sorting_bin":
            index = int(instance["name"][-1])-1
            label = ("cubes", "cuboids", "cylinders")[index]
            paint("plane", (.05, .011, .001), (.07, -.00005, .0125),
                  quaternion=(2**-.5, 2**-.5, 0, 0), label=label)
    return result


def build_scene(description, stage, seed, output, layout_path=None, randomized=False, cameras=("external", "wrist")):
    if stage not in ("continuous", "all"):
        raise ValueError("Use stage:=continuous; individual level scenes would reset the shared workspace")
    description, output = Path(description), Path(output)
    assets = description / "mjcf/competition"
    default_layout = "layout_randomized.json" if randomized else "layout.json"
    layout = json.loads(Path(layout_path or assets / default_layout).read_text())
    from panda_bringup.workspace import validate_layout
    validate_layout(layout)
    geometry = json.loads((assets / "geometry.json").read_text())
    if not all(math.isclose(a,b,abs_tol=1e-6) for a,b in zip(layout["carrier_size"],geometry["parts"]["C0_carrier"]["size_m"][:2])):
        raise ValueError("Carrier footprint must match the exported CAD")
    if layout["units"] != "m" or layout["surface_height"] != 0:
        raise ValueError("Layout must use metres and a surface at the Panda base plane (Z=0)")
    if not math.isclose(layout["drawer_joint"]["stroke"], geometry["drawer_stroke_m"], abs_tol=1e-9):
        raise ValueError("Drawer joint travel must match the exported printed end-stop geometry")
    friction = [layout["contact"][key] for key in ("sliding_friction", "torsional_friction", "rolling_friction")]
    if not all(math.isfinite(v) and v >= 0 for v in friction) or layout["density_kg_m3"] <= 0:
        raise ValueError("Contact coefficients and density are invalid")
    for key in ("carrier", "drawer"):
        assert all(math.isfinite(v) for v in layout[key]["position"]+[layout[key]["yaw"]])
    root = ET.Element("mujoco", model=f"Panda competition {stage}")
    ET.SubElement(root, "include", file=str(description / "mjcf/panda.xml"))
    ET.SubElement(root, "option", cone="elliptic", impratio="10")
    ET.SubElement(root, "compiler", meshdir=str(description / "mjcf/assets"))
    ET.SubElement(root, "statistic", center="0.3 -0.04 0.25", extent="1.8")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", azimuth="135", elevation="-40")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.65 0.65 0.65")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 .01", rgba=".23 .26 .29 1",
                  friction=numbers(friction))
    instances = []
    def fixture(name, config, parts):
        position, quat = placement(config)
        instances.append({"name": "competition_"+name, "parts": parts, "position": position,
                          "quaternion": quat, "color": COLORS["fixture"], "dynamic": False})
    part = lambda name, offset=(0, 0, 0): {"type": name, "offset": list(offset)}
    for index, config in enumerate(layout["bins"]):
        fixture(f"B1_{index+1}", config, [part("B1_sorting_bin")])
    fixture("C", layout["carrier"], [part("C0_carrier")]+[part(module["type"],module["offset"]) for module in geometry["modules"]])
    fixture("D", layout["drawer"], [part(name) for name in
            ("D0_case", "D0_roof", "D3_guide_left", "D3_guide_right", "D3_skid", "D3_stop_bushing")]+[part("D3_skid", (.120, 0, 0))])
    position, quat = placement(layout["drawer"])
    instances.append({"name": "competition_drawer", "parts": [part("D1_drawer"), part("D2_handle")],
                      "position": position, "quaternion": quat, "color": COLORS["fixture"],
                      "dynamic": True, "joint": "drawer_slide"})
    objects = object_poses(stage, seed, layout)
    instances.extend(objects)
    loaded = set()
    for label in ("cubes", "cuboids", "cylinders", "red", "green", "blue"):
        ET.SubElement(asset, "texture", name="label_"+label, type="2d", file=str(assets / "labels" / f"{label}.png"))
        ET.SubElement(asset, "material", name="label_"+label, texture="label_"+label, rgba="1 1 1 1")
    key = ET.SubElement(ET.SubElement(root, "keyframe"), "key", name="competition_home")
    panda = ET.parse(description / "mjcf/panda.xml").getroot()
    home = panda.find("keyframe/key[@name='home']")
    initial = home.get("qpos").split()
    key.set("ctrl", home.get("ctrl"))
    for instance in instances:
        body = ET.SubElement(world, "body", name=instance["name"], pos=numbers(instance["position"]),
                             quat=numbers(instance["quaternion"]))
        if instance.get("joint"):
            joint = layout["drawer_joint"]
            ET.SubElement(body, "joint", name="drawer_slide", type="slide", axis="0 -1 0",
                          range=numbers((0, joint["stroke"])), damping=str(joint["damping"]),
                          frictionloss=str(joint["frictionloss"]), armature="0.001")
            initial.append("0")
        elif instance["dynamic"]:
            ET.SubElement(body, "freejoint", name=instance["name"]+"_free")
            initial.extend(map(str, instance["position"]+instance["quaternion"]))
        for item in instance["parts"]:
            definition = geometry["parts"][item["type"]]
            for filename in [definition["visual"], *definition["collision"]]:
                mesh_name = filename.replace("/", "_").replace(".obj", "")
                if mesh_name not in loaded:
                    ET.SubElement(asset, "mesh", name=mesh_name, file=str(assets / filename))
                    loaded.add(mesh_name)
                is_visual = filename == definition["visual"]
                ET.SubElement(body, "geom", name=f"{instance['name']}_{len(body)}", type="mesh", mesh=mesh_name,
                              pos=numbers(item["offset"]), mass="0", group="2" if is_visual else "3",
                              contype="0" if is_visual else "1", conaffinity="0" if is_visual else "1",
                              rgba=numbers(instance["color"] if is_visual else [0.5, 0.5, 0.5, 0]),
                              friction=numbers(friction), condim="4",
                              solref="0.004 1", solimp="0.95 0.99 0.001")
        instance["decorations"] = decorations(instance, stage)
        for index, paint in enumerate(instance["decorations"]):
            attributes = {"material": "label_"+paint["label"]} if paint["label"] else {}
            ET.SubElement(body, "geom", name=f"{instance['name']}_paint_{index}", type=paint["shape"],
                          size=numbers(paint["size"]), pos=numbers(paint["position"]), quat=numbers(paint["quaternion"]),
                          rgba=numbers(paint["color"]), mass="0", contype="0", conaffinity="0", group="2", **attributes)
        if instance["dynamic"]:
            masses = []
            centers = []
            for item in instance["parts"]:
                definition = geometry["parts"][item["type"]]
                mass = definition["mass_kg"]*layout["density_kg_m3"]/geometry["density_assumption_kg_m3"]
                masses.append(mass)
                centers.append([a+b for a, b in zip(definition["center_of_mass_m"], item["offset"])])
            total = sum(masses)
            center = [sum(m*c[i] for m, c in zip(masses, centers))/total for i in range(3)]
            inertia = [[0.0]*3 for _ in range(3)]
            for item, mass, c in zip(instance["parts"], masses, centers):
                definition = geometry["parts"][item["type"]]
                delta = [c[i]-center[i] for i in range(3)]
                for i in range(3):
                    for j in range(3):
                        inertia[i][j] += definition["inertia_kg_m2"][i][j]*mass/definition["mass_kg"]
                        inertia[i][j] += mass*((sum(v*v for v in delta) if i == j else 0)-delta[i]*delta[j])
            override = layout["mass_overrides_kg"].get(instance["name"], total)
            if not math.isfinite(override) or override <= 0:
                raise ValueError("Mass overrides must be finite and positive")
            full = [inertia[i][j]*override/total for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))]
            ET.SubElement(body, "inertial", mass=str(override), pos=numbers(center), fullinertia=numbers(full))
    key.set("qpos", " ".join(initial))
    origin = layout["object_area"]["origin"]
    width, depth = layout["object_area"]["size"]
    ET.SubElement(world, "geom", name="A0_start_mat", type="box", pos=numbers((origin[0]+width/2, origin[1]+depth/2, .00005)),
                  size=numbers((width/2, depth/2, .00005)), rgba=".85 .87 .88 1", contype="0", conaffinity="0", group="2")
    tower=layout["tower"]
    tx,ty,tz=tower["position"];tw,td=tower["size"]
    for i,(px,py,sx,sy) in enumerate([(tx+tw/2,ty,tw/2,.001),(tx+tw/2,ty+td,tw/2,.001),
                                     (tx,ty+td/2,.001,td/2),(tx+tw,ty+td/2,.001,td/2)]):
        ET.SubElement(world,"geom",name=f"tower_target_{i}",type="box",pos=numbers((px,py,.00015)),
                      size=numbers((sx,sy,.00005)),rgba="1 1 1 1",contype="0",conaffinity="0",group="2")
    state = {"stage": stage, "seed": seed, "layout": layout, "instances": instances,
             "geometry": str(assets / "geometry.json"), "surface_height": 0, "randomized": randomized, "continuous": True, "modules": geometry["modules"]}
    output.mkdir(parents=True, exist_ok=True)
    if cameras:
        from panda_bringup.cameras import add_cameras
        add_cameras(world, panda, state, cameras)
        camera_robot = output / "panda_cameras.xml"
        ET.ElementTree(panda).write(camera_robot, encoding="unicode")
        root.find("include").set("file", str(camera_robot.resolve()))
    scene = output / "competition.xml"
    ET.indent(root)
    ET.ElementTree(root).write(scene, encoding="unicode")
    from panda_bringup.randomization import validate_scene
    validate_scene(scene)
    state_path = output / "competition.json"
    state_path.write_text(json.dumps(state, indent=2)+"\n")
    return scene, state_path
