import math


def fixture_regions(layout):
    regions = [("C", layout["carrier"], tuple(layout["carrier_size"])),
               ("D", layout["drawer"], (.180, .160))]
    regions.extend((f"B1_{i+1}", config, (.140, .110)) for i, config in enumerate(layout["bins"]))
    drawer = layout["drawer"]
    x, y, z = drawer["position"]
    yaw = drawer["yaw"]
    regions.append(("D_opening", {"position": [x+.135*math.sin(yaw), y-.135*math.cos(yaw), z],
                                  "yaw": yaw}, (.180, .135)))
    return regions


def point_rectangle_distance(x, y, config, size):
    dx, dy = x-config["position"][0], y-config["position"][1]
    a = config["yaw"]
    local_x, local_y = math.cos(a)*dx+math.sin(a)*dy, -math.sin(a)*dx+math.cos(a)*dy
    return math.hypot(max(-local_x, local_x-size[0], 0), max(-local_y, local_y-size[1], 0))


def corners(config, size):
    c, s = math.cos(config["yaw"]), math.sin(config["yaw"])
    px, py = config["position"][:2]
    return [(px+c*x-s*y, py+s*x+c*y) for x, y in ((0, 0), (size[0], 0), size, (0, size[1]))]


def rectangle_distance(first, second):
    a, b = corners(*first), corners(*second)
    separated = False
    for config, _ in (first, second):
        for angle in (config["yaw"], config["yaw"]+math.pi/2):
            axis = (math.cos(angle), math.sin(angle))
            pa, pb = [[x*axis[0]+y*axis[1] for x, y in polygon] for polygon in (a, b)]
            separated |= max(pa) < min(pb) or max(pb) < min(pa)
    if not separated:
        return 0.0
    return min([point_rectangle_distance(x, y, *second) for x, y in a]+
               [point_rectangle_distance(x, y, *first) for x, y in b])


def validate_layout(layout):
    validate_tower(layout)
    area = layout["object_area"]
    regions = [("A0", {"position": area["origin"], "yaw": 0.0}, area["size"]), *fixture_regions(layout)]
    for name, config, size in regions:
        if not all(math.isfinite(v) for v in [*config["position"], config["yaw"], *size]) or min(size) <= 0:
            raise ValueError(f"Invalid workspace region: {name}")
        if point_rectangle_distance(0, 0, config, size) < .175:
            raise ValueError(f"{name} encroaches on the robot-base clearance")
    for i, (name, config, size) in enumerate(regions):
        for other, other_config, other_size in regions[i+1:]:
            if {name, other} == {"D", "D_opening"}:
                continue
            pair = {name, other}
            if "D" in pair and "A0" in pair:
                required = .120
            else:
                required = .040 if "A0" in pair else .025
            distance = rectangle_distance((config, size), (other_config, other_size))
            if distance < required-1e-9:
                raise ValueError(f"Workspace regions {name} and {other}: {distance*1000:.1f} mm gap; "
                                 f"at least {required*1000:.0f} mm required")


def validate_tower(layout):
    area, tower = layout["object_area"], layout["tower"]
    ox,oy,_=area["origin"]
    for x,y in corners(tower,tower["size"]):
        if not ox+.005<=x<=ox+area["size"][0]-.005 or not oy+.005<=y<=oy+area["size"][1]-.005:
            raise ValueError("Tower destination must stay inside A0 with 5 mm edge margin")
