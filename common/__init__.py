"""Shared helpers for OutdoorNav raw-ROS examples.

The rule for this folder is: no clearpath_outdoornav_api_lib imports anywhere
in examples/, ever. Helpers in this package stay at the rclpy level so callers
remain easy to audit by reading the service / topic / action names directly.
"""
