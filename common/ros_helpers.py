"""rclpy boilerplate shared across examples.

Stays at the rclpy level — these helpers never wrap an OutdoorNav interface.
Reading any example, every service / topic / action name is still visible at
the call site.
"""

from __future__ import annotations
from typing import Any

import rclpy
from rclpy.node import Node


def wait_for_service(node: Node, client, name: str, timeout_each: float = 1.0) -> None:
    """Block until a service client connects, logging progress."""
    node.get_logger().info(f"waiting for service {name} …")
    while not client.wait_for_service(timeout_sec=timeout_each):
        if not rclpy.ok():
            raise RuntimeError("rclpy shutdown while waiting for service")
        node.get_logger().info(f"  still waiting for {name}")
    node.get_logger().info(f"  ✓ {name} available")


def wait_for_action(node: Node, client, name: str, timeout_each: float = 1.0) -> None:
    """Block until an action server connects, logging progress."""
    node.get_logger().info(f"waiting for action server {name} …")
    while not client.wait_for_server(timeout_sec=timeout_each):
        if not rclpy.ok():
            raise RuntimeError("rclpy shutdown while waiting for action server")
        node.get_logger().info(f"  still waiting for {name}")
    node.get_logger().info(f"  ✓ {name} available")


def call_service(node: Node, client, request: Any, timeout_sec: float = 30.0) -> Any:
    """Synchronous service call: spin until result or timeout. Returns the response."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        raise TimeoutError(f"service call timed out after {timeout_sec}s")
    return future.result()
