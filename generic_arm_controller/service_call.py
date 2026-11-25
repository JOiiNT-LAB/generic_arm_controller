"""
Call a ROS service from a reentrant callback and wait with a timeout.

rclpy's call() would deadlock from a callback of the same node; the executor nodes spin on a
MultiThreadedExecutor, so call_async() plus a short sleep loop lets the response arrive on
another thread.
"""

import time


def call_with_timeout(client, request, timeout_s):
    """Return (True, result) when the call completes, (False, None) on timeout."""
    future = client.call_async(request)
    start = time.time()
    while not future.done():
        time.sleep(0.01)
        if time.time() - start > timeout_s:
            return False, None
    return True, future.result()
