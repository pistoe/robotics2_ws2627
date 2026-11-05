import copy
import json
from pathlib import Path
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, ObjectColor, PlanningScene, PlanningSceneComponents, PlanningSceneWorld, RobotState
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

def pose(position, quaternion):
    return Pose(position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])), orientation=Quaternion(w=float(quaternion[0]), x=float(quaternion[1]), y=float(quaternion[2]), z=float(quaternion[3])))

def load_mesh(path):
    vertices, triangles = ([], [])
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append(Point(x=float(fields[1]), y=float(fields[2]), z=float(fields[3])))
        elif fields[0] == 'f':
            triangles.append(MeshTriangle(vertex_indices=[int(v.split('/')[0]) - 1 for v in fields[1:]]))
    return Mesh(vertices=vertices, triangles=triangles)

class CompetitionScene(Node):

    def __init__(self):
        super().__init__('competition_scene')
        path = Path(self.declare_parameter('scene_description', '').value)
        self.scene = json.loads(path.read_text())
        self.geometry_path = Path(self.scene['geometry'])
        self.geometry = json.loads(self.geometry_path.read_text())['parts']
        self.instances = {instance['name']: instance for instance in self.scene['instances'] if not instance['dynamic']}
        self.poses = {name: pose(item['position'], item['quaternion']) for name, item in self.instances.items()}
        self.last_poses = {}
        self.acm_ready = not self.declare_parameter('allow_fixture_support_contacts', False).value
        self.support_matrix = None
        self.attached = set()
        self.pending_add = set(self.instances)
        self.objects, self.markers = ({}, [])
        cache = {}
        for name, item in self.instances.items():
            meshes, mesh_poses = ([], [])
            for part in item['parts']:
                definition = self.geometry[part['type']]
                for filename in definition['collision']:
                    if filename not in cache:
                        cache[filename] = load_mesh(self.geometry_path.parent / filename)
                    meshes.append(cache[filename])
                    mesh_poses.append(pose(part['offset'], [1, 0, 0, 0]))
                marker = Marker(header=Header(frame_id=name), ns='competition', id=len(self.markers), type=Marker.MESH_RESOURCE, action=Marker.ADD, pose=pose(part['offset'], [1, 0, 0, 0]), scale=Vector3(x=1.0, y=1.0, z=1.0), color=ColorRGBA(r=float(item['color'][0]), g=float(item['color'][1]), b=float(item['color'][2]), a=1.0), mesh_resource='package://panda_description/mjcf/competition/' + definition['visual'])
                self.markers.append((name, marker))
            for paint in item.get('decorations', []):
                size = paint['size']
                shape = {'cylinder': Marker.CYLINDER, 'box': Marker.CUBE, 'plane': Marker.TEXT_VIEW_FACING}[paint['shape']]
                scale = [2 * size[0], 2 * size[0], 2 * size[1]] if paint['shape'] == 'cylinder' else [2 * v for v in size]
                if paint['shape'] == 'plane':
                    scale = [0.004, 0.004, 0.004]
                self.markers.append((name, Marker(ns='competition', id=len(self.markers), type=shape, action=Marker.ADD, pose=pose(paint['position'], paint['quaternion']), scale=Vector3(x=float(scale[0]), y=float(scale[1]), z=float(scale[2])), color=ColorRGBA(r=float(paint['color'][0]), g=float(paint['color'][1]), b=float(paint['color'][2]), a=1.0), text=paint['label'].upper())))
            self.objects[name] = CollisionObject(header=Header(frame_id='world'), id=name, pose=self.poses[name], meshes=meshes, mesh_poses=mesh_poses, operation=CollisionObject.ADD)
        self.world_publisher = self.create_publisher(PlanningScene, 'planning_scene', 5)
        self.marker_publisher = self.create_publisher(MarkerArray, 'competition/markers', 5)
        self.scene_client = self.create_client(GetPlanningScene, 'get_planning_scene')
        self.scene_future = None
        self.create_timer(0.5, self.attachments)
        self.create_timer(0.05, self.publish)
        self.get_logger().info(f"Competition stage {self.scene['stage']}: {len(self.instances)} bodies; fixed fixtures only")

    def attachments(self):
        if not self.scene_client.service_is_ready() or (self.scene_future and (not self.scene_future.done())):
            return
        components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        if not self.acm_ready:
            components |= PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        request = GetPlanningScene.Request(components=PlanningSceneComponents(components=components))
        self.scene_future = self.scene_client.call_async(request)
        self.scene_future.add_done_callback(self.attachment_result)

    def attachment_result(self, future):
        result = future.result()
        if result is None:
            return
        if not self.acm_ready:
            matrix = result.scene.allowed_collision_matrix
            fixed = [name for name, item in self.instances.items() if not item['dynamic']]
            for name in ['panda_floor', *fixed]:
                if name not in matrix.entry_names:
                    matrix.entry_names.append(name)
                    for entry in matrix.entry_values:
                        entry.enabled.append(False)
                    matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
            floor = matrix.entry_names.index('panda_floor')
            for name in fixed:
                index = matrix.entry_names.index(name)
                matrix.entry_values[floor].enabled[index] = True
                matrix.entry_values[index].enabled[floor] = True
            self.support_matrix, self.acm_ready = (matrix, True)
        attached = {item.object.id for item in result.scene.robot_state.attached_collision_objects if item.object.id in self.instances}
        self.pending_add.update(self.attached - attached)
        self.pending_add.difference_update(attached)
        self.attached = attached

    def publish(self):
        changes, colors = ([], [])
        if self.acm_ready and self.world_publisher.get_subscription_count():
            for name, item in self.instances.items():
                if name in self.attached:
                    continue
                current = self.poses[name]
                if name in self.pending_add:
                    obj = copy.deepcopy(self.objects[name])
                    obj.pose = current
                    changes.append(obj)
                    colors.append(ObjectColor(id=name, color=ColorRGBA(r=float(item['color'][0]), g=float(item['color'][1]), b=float(item['color'][2]), a=1.0)))
                    self.pending_add.discard(name)
                self.last_poses[name] = current
            if changes:
                update = PlanningScene(is_diff=True, robot_state=RobotState(is_diff=True), world=PlanningSceneWorld(collision_objects=changes), object_colors=colors)
                if self.support_matrix is not None:
                    update.allowed_collision_matrix = self.support_matrix
                    self.support_matrix = None
                self.world_publisher.publish(update)
        if not self.marker_publisher.get_subscription_count():
            return
        markers = []
        for name, original in self.markers:
            marker = copy.deepcopy(original)
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            body = self.poses[name]
            offset = original.pose.position
            q = body.orientation
            u = (q.x, q.y, q.z)
            v = (offset.x, offset.y, offset.z)
            cross = lambda a, b: (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
            uv = cross(u, v)
            uuv = cross(u, uv)
            xyz = [v[i] + 2 * (q.w * uv[i] + uuv[i]) for i in range(3)]
            local = original.pose.orientation
            rotation = Quaternion(w=q.w * local.w - q.x * local.x - q.y * local.y - q.z * local.z, x=q.w * local.x + q.x * local.w + q.y * local.z - q.z * local.y, y=q.w * local.y - q.x * local.z + q.y * local.w + q.z * local.x, z=q.w * local.z + q.x * local.y - q.y * local.x + q.z * local.w)
            marker.pose = Pose(position=Point(x=body.position.x + xyz[0], y=body.position.y + xyz[1], z=body.position.z + xyz[2]), orientation=rotation)
            markers.append(marker)
        self.marker_publisher.publish(MarkerArray(markers=markers))

def main():
    from panda_bringup.node_lifecycle import spin_node
    spin_node(CompetitionScene)
