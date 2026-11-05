import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def spin_node(factory):
    stopping = False

    def interrupt(signum, frame):
        nonlocal stopping
        stopping = True

    # Keep the ROS context alive until subscription callbacks have unwound.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    executor = rclpy.get_global_executor()
    try:
        node = factory()
        executor.add_node(node)
        while not stopping and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
