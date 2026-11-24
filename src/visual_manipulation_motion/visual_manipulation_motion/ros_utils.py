"""Blocking helpers for rclpy futures, for code that runs in a worker thread while the node is
spun by a MultiThreadedExecutor (same pattern as openarm_sorting_manipulation.ros_utils)."""
import threading


class RosTimeout(RuntimeError):
    pass


def wait_future(future, timeout_sec, what='request'):
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout_sec):
        future.cancel()
        raise RosTimeout(f'{what} timed out after {timeout_sec:.1f} s')
    exc = future.exception()
    if exc is not None:
        raise exc
    return future.result()


def call_service(client, request, timeout_sec, what=None):
    what = what or client.srv_name
    if not client.wait_for_service(timeout_sec=min(timeout_sec, 10.0)):
        raise RosTimeout(f'service {what} not available')
    return wait_future(client.call_async(request), timeout_sec, what)


def run_action(client, goal, timeout_sec, what='action'):
    """Send an action goal and wait for its result (returns the wrapped result response)."""
    if not client.wait_for_server(timeout_sec=min(timeout_sec, 10.0)):
        raise RosTimeout(f'action server {what} not available')
    handle = wait_future(client.send_goal_async(goal), 10.0, f'{what} goal')
    if not handle.accepted:
        raise RuntimeError(f'{what} goal rejected')
    return wait_future(handle.get_result_async(), timeout_sec, f'{what} result')
