import time

from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import ListControllers, SwitchController
from mujoco_ros2_control_msgs.srv import ResetWorld, SetPause, StepSimulation
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger


class SimulationRuntime(Node):
    def __init__(self):
        super().__init__('simulation')
        self.group = ReentrantCallbackGroup()
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.arm_enabled = self.declare_parameter('arm_enabled', True).value
        self.resume_on_start = self.declare_parameter('resume_on_start', True).value
        self.required = {'joint_state_broadcaster'} | ({'panda_arm_controller'} if self.arm_enabled else set())
        self.busy = False
        self.started = False
        self.deadline = time.monotonic() + 65
        self.ready = self.create_publisher(Bool, '~/ready',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready.publish(Bool(data=False))
        self.list_controllers = self.client(ListControllers, 'controller_manager/list_controllers')
        self.switch = self.client(SwitchController, 'controller_manager/switch_controller')
        self.pause = self.client(SetPause, 'mujoco_ros2_control_node/set_pause')
        self.release = self.client(Trigger, 'mujoco_ros2_control_node/release_control')
        self.reset_world = self.client(ResetWorld, 'mujoco_ros2_control_node/reset_world')
        self.step = self.client(StepSimulation, 'mujoco_ros2_control_node/step_simulation')
        self.cancel = self.client(CancelGoal, 'panda_arm_controller/follow_joint_trajectory/_action/cancel_goal')
        self.create_service(Trigger, '~/reset', self.reset, callback_group=self.group)
        self.create_subscription(Empty, 'mujoco_ros2_control_node/world_reset', self.external_reset, 10,
                                 callback_group=self.group)
        self.startup_timer = self.create_timer(.1, self.startup, clock=self.wall_clock, callback_group=self.group)

    def client(self, kind, name):
        return self.create_client(kind, name, callback_group=self.group)

    async def call(self, client, request, timeout=12.):
        if not client.service_is_ready():
            raise RuntimeError(f'Service is unavailable: {client.srv_name}')
        future = client.call_async(request)
        def expired():
            if not future.done():
                future.set_exception(TimeoutError(client.srv_name))
        timer = self.create_timer(timeout, expired, clock=self.wall_clock, callback_group=self.group)
        try:
            result = await future
            if hasattr(result, 'success') and not result.success:
                raise RuntimeError(f'{client.srv_name}: {result.message}')
            return result
        finally:
            self.destroy_timer(timer)

    async def startup(self):
        if self.started or self.busy:
            return
        if time.monotonic() > self.deadline:
            raise RuntimeError('Controller readiness timed out; physics remains gated')
        if not all(client.service_is_ready() for client in (self.list_controllers, self.pause, self.release)):
            return
        self.busy = True
        try:
            result = await self.call(self.list_controllers, ListControllers.Request())
            active = {controller.name for controller in result.controller if controller.state == 'active'}
            if not self.required <= active:
                return
            if self.arm_enabled:
                await self.call(self.release, Trigger.Request())
                await self.call(self.pause, SetPause.Request(paused=not self.resume_on_start))
            self.started = True
            self.startup_timer.cancel()
            self.ready.publish(Bool(data=True))
            self.get_logger().info('Requested controllers are active; startup barrier complete')
        finally:
            self.busy = False

    async def switch_arm(self, activate):
        request = SwitchController.Request(strictness=SwitchController.Request.STRICT)
        request.activate_controllers = ['panda_arm_controller'] if activate else []
        request.deactivate_controllers = [] if activate else ['panda_arm_controller']
        request.timeout.sec = 5
        result = await self.call(self.switch, request)
        if not result.ok:
            raise RuntimeError('Arm controller switch failed')

    async def recover(self, restore_world):
        if not self.started or self.busy or not self.arm_enabled:
            return False, 'Simulation is not ready for a reset'
        self.busy = True
        self.ready.publish(Bool(data=False))
        try:
            await self.call(self.pause, SetPause.Request(paused=True))
            cancelled = await self.call(self.cancel, CancelGoal.Request())
            if cancelled.return_code not in (CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID):
                raise RuntimeError(f'Goal cancellation failed: {cancelled.return_code}')
            await self.switch_arm(False)
            if restore_world:
                await self.call(self.reset_world, ResetWorld.Request())
            await self.switch_arm(True)
            await self.call(self.release, Trigger.Request())
            # Advance one broadcaster interval so the reset has a fresh TF timestamp.
            await self.call(self.step, StepSimulation.Request(steps=10))
            self.ready.publish(Bool(data=True))
            self.get_logger().info('Reset recovery complete; arm holds reset state with physics paused')
            return True, 'Reset complete; physics is paused and old arm goals are retired'
        except Exception as error:
            self.get_logger().error(f'Reset recovery failed; physics stays paused: {error}')
            return False, str(error)
        finally:
            self.busy = False

    async def reset(self, request, response):
        response.success, response.message = await self.recover(True)
        return response

    async def external_reset(self, message):
        if self.started and not self.busy:
            await self.recover(False)


def main():
    from panda_bringup.node_lifecycle import spin_node
    spin_node(SimulationRuntime)


if __name__ == '__main__':
    main()
