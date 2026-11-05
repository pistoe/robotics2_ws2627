import math
import random

from panda_bringup.workspace import fixture_regions, point_rectangle_distance, rectangle_distance


FOOTPRINTS = {"O1": (.040, .040), "O2": (.050, .030), "O3": (.036, .036),
              "O4": (.060, .050), "O5": (.032, .032), "O6": (.032, .032), "O7": (.140, .070), "O8": (.140, .070)}
CENTERS = {"O1": (.020, .020), "O2": (.025, .015), "O3": (0, 0),
           "O4": (.030, .025), "O5": (.016, .016), "O6": (.016, .016), "O7": (.070, .035), "O8": (.070, .035)}
CLEARANCE = .020
EDGE_MARGIN = .010


def radius(item):
    kind = item["name"].split("_")[1]
    return .018 if kind == "O3" else math.hypot(*FOOTPRINTS[kind])/2


def set_pose(item, center, yaw, z=0.0):
    kind = item["name"].split("_")[1]
    x, y = CENTERS[kind]
    item["position"] = [center[0]-math.cos(yaw)*x+math.sin(yaw)*y,
                        center[1]-math.sin(yaw)*x-math.cos(yaw)*y, z]
    item["quaternion"] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]


def randomize(objects, stage, seed, layout):
    rng = random.Random(seed)
    table = [item for item in objects if item["name"] != "competition_O6_1"]
    origin, size = layout["object_area"]["origin"], layout["object_area"]["size"]
    if len(origin)!=3 or len(size)!=2 or not all(math.isfinite(v) for v in origin+size) or min(size)<=0:
        raise ValueError("Object area must have finite coordinates and positive dimensions")
    if min(size)<.1:
        raise ValueError("Object area is too small for the complete inventory")
    tower=layout["tower"]
    fixed=[*fixture_regions(layout),("tower",tower,tower["size"])]
    for _ in range(80):
        order=sorted(table,key=radius,reverse=True)
        placed=[]
        for item in order:
            dimensions=FOOTPRINTS[item["name"].split("_")[1]]
            for _ in range(1200):
                yaw=rng.uniform(-math.pi,math.pi)
                c,s=abs(math.cos(yaw)),abs(math.sin(yaw))
                mx=(c*dimensions[0]+s*dimensions[1])/2+EDGE_MARGIN
                my=(s*dimensions[0]+c*dimensions[1])/2+EDGE_MARGIN
                if size[0]<=2*mx or size[1]<=2*my:
                    continue
                center=(rng.uniform(origin[0]+mx,origin[0]+size[0]-mx),
                        rng.uniform(origin[1]+my,origin[1]+size[1]-my))
                set_pose(item,center,yaw,origin[2])
                config={"position":[center[0]-math.cos(yaw)*dimensions[0]/2+math.sin(yaw)*dimensions[1]/2,
                                    center[1]-math.sin(yaw)*dimensions[0]/2-math.cos(yaw)*dimensions[1]/2,origin[2]],"yaw":yaw}
                if any(rectangle_distance((config,dimensions),(other,other_size))<CLEARANCE
                       for other,other_size in placed):
                    continue
                if any(rectangle_distance((config,dimensions),(other,other_size))<CLEARANCE
                       for _,other,other_size in fixed):
                    continue
                placed.append((config,dimensions))
                break
            else:
                break
        else:
            break
    else:
        raise ValueError("Cannot place all fourteen table parts with 20 mm clearance and a clear tower destination")
    drawer=layout["drawer"]
    local_x,local_y=rng.uniform(.055,.065),rng.uniform(.028,.032)
    a=drawer["yaw"]
    center=[drawer["position"][0]+math.cos(a)*local_x-math.sin(a)*local_y,
            drawer["position"][1]+math.sin(a)*local_x+math.cos(a)*local_y]
    item=next(item for item in objects if item["name"]=="competition_O6_1")
    yaw=a+rng.randrange(4)*math.pi/2+rng.uniform(-math.pi/15,math.pi/15)
    set_pose(item,center,yaw,drawer["position"][2]+.019)
    return objects


def validate_scene(scene_path):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("competition_home").id)
    mujoco.mj_forward(model, data)
    for contact in data.contact:
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or "world"
                 for g in contact.geom]
        if contact.dist < -.00005 and any(name.startswith("competition_O") for name in names):
            raise ValueError(f"Randomized scene has an initial penetration: {names}, {contact.dist:.6f} m")
