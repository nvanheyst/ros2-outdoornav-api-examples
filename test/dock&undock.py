import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from clearpath_dock_msgs.action import NetworkDock, Undock

class DockClient(Node):
    def __init__(self):
        super().__init__('dock_client')

        # Create action clients
        self.network_dock_client = ActionClient(self, NetworkDock, '/a300_00003/autonomy/network_dock')
        self.undock_client = ActionClient(self, Undock, '/a300_00003/autonomy/local_undock')

        # Start the docking sequence
        self.send_network_dock_goal()

    def send_network_dock_goal(self):
        goal_msg = NetworkDock.Goal()
        goal_msg.dock_name = 'fake_dock1'
        goal_msg.network_uuid = 'c85ee84f-3af5-46d9-935c-b54c1939f159'

        self.network_dock_client.wait_for_server()
        self.get_logger().info('Sending NetworkDock goal...')
        future = self.network_dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.network_dock_response)

    def network_dock_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('NetworkDock goal rejected')
            return

        self.get_logger().info('NetworkDock goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.network_dock_result)

    def network_dock_result(self, future):
        result = future.result().result
        self.get_logger().info(f'Dock Result: success={result.success}, message={result.message}')

        if result.success:
            self.get_logger().info('Docking succeeded — proceeding to undock...')
            self.send_undock_goal()
        else:
            self.get_logger().warn('Docking failed — skipping undock.')
            rclpy.shutdown()

    def send_undock_goal(self):
        goal_msg = Undock.Goal()
        goal_msg.dock_name = 'fake_dock1'

        self.undock_client.wait_for_server()
        self.get_logger().info('Sending Undock goal...')
        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_response)

    def undock_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Undock goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Undock goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result)

    def undock_result(self, future):
        result = future.result().result
        self.get_logger().info(f'Undock Result: success={result.success}, message={result.message}, elapsed_time={result.elapsed_time}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    dock_client = DockClient()
    rclpy.spin(dock_client)
    dock_client.destroy_node()

if __name__ == '__main__':
    main()
