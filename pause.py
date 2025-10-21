import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 
from rclpy.task import Future
from geometry_msgs.msg import Twist, TwistStamped
import time
from std_srvs.srv import SetBool, Trigger 
from clearpath_navigation_msgs.action import ExecuteNetworkGoToPOI as GoToPoi 


class MissionPlanner(Node):
    """
    Executes a sequential mission designed for a planned interruption to showcase API usage.
    The mission flow is as follows:
    1. Start GoToPOI (non-blocking).
    2. Wait 5 seconds to interrupt navigation mid-route.
    3. Pause autonomy.
    4. Execute teleop maneuvers (turn + drive) while paused.
    5. Resume autonomy
    6. BLOCK and wait for GoToPOI to finally complete.
"""
    # -- Topic Constants --
    ROBOT_NAMESPACE = '/a300_00003'
    ACTION_GO_TO_POI    = f'{ROBOT_NAMESPACE}/autonomy/network_goto_poi'
    CMD_VEL_TOPIC       = f'{ROBOT_NAMESPACE}/ui_teleop/cmd_vel'
    
    SERVICE_PAUSE       = f'{ROBOT_NAMESPACE}/control_selection/pause'
    SERVICE_RESUME      = f'{ROBOT_NAMESPACE}/control_selection/resume'
    SERVICE_STOP        = f'{ROBOT_NAMESPACE}/autonomy/stop'
    
    # -- Mission Parameters --
    POI_ID = '0caf3108-23c9-4e04-ad65-c8131fa073f0'
    MAP_ID = 'c85ee84f-3af5-46d9-935c-b54c1939f159'

    # -- Teleop Constants (Tune these for your robot) --
    TURN_ANGLE_RAD = 3.14159
    TURN_VEL_RAD_S = 0.5
    TURN_DURATION = TURN_ANGLE_RAD / TURN_VEL_RAD_S 
    
    DRIVE_DISTANCE_M = 1.0
    DRIVE_VEL_M_S = 0.3
    DRIVE_DURATION = DRIVE_DISTANCE_M / DRIVE_VEL_M_S 
    
    PUBLISH_RATE = 0.05 
    
    def __init__(self):
        super().__init__('mission_planner')
        self.get_logger().info(f'Mission Planner started for robot: {self.ROBOT_NAMESPACE}')
        
        self.poi_client = ActionClient(self, GoToPoi, self.ACTION_GO_TO_POI)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        self.pause_client = self.create_client(SetBool, self.SERVICE_PAUSE)
        self.resume_client = self.create_client(SetBool, self.SERVICE_RESUME)
        self.autonomy_stop_client = self.create_client(Trigger, self.SERVICE_STOP)
        
        self._wait_for_clients()
        
    def _wait_for_clients(self):
        """Blocks until all necessary ROS servers are available."""
        self.get_logger().info('Waiting for services and action servers...')
        
        while not all([
            self.poi_client.wait_for_server(timeout_sec=1.0),
            self.pause_client.wait_for_service(timeout_sec=1.0),
            self.resume_client.wait_for_service(timeout_sec=1.0),
            self.autonomy_stop_client.wait_for_service(timeout_sec=1.0),
        ]):
            self.get_logger().info('Still waiting...')
            time.sleep(1.0)
            
        

    def publish_twist(self, linear_x, angular_z):
        """Publishes a TwistStamped message for direct motion control."""
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = twist
        
        self.cmd_vel_publisher.publish(twist_stamped)


    def execute_teleop_motion(self, linear_vel, angular_vel, duration, action_name):
        """Publishes teleop commands for a specified duration"""
        self.get_logger().info(f'{action_name} started')
        
        start_time_s = time.time()
        self.publish_twist(linear_vel, angular_vel)
        
        end_wait_time = time.time() + duration
        while time.time() < end_wait_time:
            self.publish_twist(linear_vel, angular_vel)
            time.sleep(self.PUBLISH_RATE)
        
        self.publish_twist(0.0, 0.0) # Stop motion
        
        self.get_logger().info(f'SUCCESS: {action_name} finished.')
        return True
        
    def execute_turn(self):
        """Executes the approximate 180-degree turn."""
        return self.execute_teleop_motion(0.0, self.TURN_VEL_RAD_S, self.TURN_DURATION, f'180 DEGREE TURN')

    def execute_straight_drive(self):
        """Executes the approximate 1-meter drive."""
        return self.execute_teleop_motion(self.DRIVE_VEL_M_S, 0.0, self.DRIVE_DURATION, f'1 METER DRIVE')

    def send_go_to_poi_goal(self, poi_uuid: str, map_uuid: str):
        """Sends GoToPOI goal and returns the future for later blocking."""
        goal_msg = GoToPoi.Goal(poi_uuid=poi_uuid, map_uuid=map_uuid)
        send_goal_future = self.poi_client.send_goal_async(goal_msg)
        self.get_logger().info('GoToPOI goal sent. Robot is now driving.')
        return send_goal_future

    def wait_for_go_to_poi_completion(self, send_goal_future: Future):
        """Blocks execution until the GoToPOI action completes after resume."""
        self.get_logger().info('Blocking: Waiting for GoToPOI to complete...')
        
        # 1. Wait for goal acceptance/re-establishment
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('FATAL: GoToPOI goal was never accepted or got rejected.')
            return False

        self.get_logger().info('GoToPOI goal accepted/re-established. Waiting for final result...')
        
        # 2. Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        # Status 4 is typically the SUCCEEDED status for Nav2/Clearpath actions
        if get_result_future.result() and get_result_future.result().status == 4:
             self.get_logger().info('SUCCESS: GoToPOI destination reached.')
             return True
        else:
            self.get_logger().error(f'FAILURE: GoToPOI failed to complete after resume. Status: {get_result_future.result().status if get_result_future.result() else "NO RESULT"}')
            return False


    def _call_set_bool_service(self, client, data: bool, action_name: str):
        """Helper for calling SetBool services (Pause/Resume)."""
        request = SetBool.Request(data=data)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'SUCCESS: {action_name} confirmed.')
            return True
        else:
            self.get_logger().error(f'FAILURE: {action_name} service call failed.')
            return False

    def _call_trigger_service(self, client, action_name: str):
        """Helper for calling Trigger services."""
        self.get_logger().info(f'Attempting to trigger {action_name}...')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'SUCCESS: {action_name} confirmed.')
            return True
        elif future.result() is not None:
             self.get_logger().error(f'FAILURE: {action_name} failed. Message: {future.result().message}')
             return False
        else:
            self.get_logger().error(f'FAILURE: {action_name} service call failed (no response).')
            return False


    def execute_mission(self):
        """The main, sequential mission logic."""
        self.get_logger().info("Starting mission...")
        
        # STEP 1: Execute GoToPOI
        poi_future = self.send_go_to_poi_goal(self.POI_ID, self.MAP_ID)
        
        # STEP 2: Wait 5 seconds to interrupt navigation mid-route
        time.sleep(10)
        self.get_logger().info('Pausing robot to interrupt GoToPOI...')
        
        # STEP 3: PAUSE Autonomy
        if not self._call_set_bool_service(self.pause_client, True, 'PAUSE'):
            self.get_logger().error("PAUSE failed. Aborting mission.")
            return

        # STEP 4: Teleop during pause (turn + drive)
        self.execute_turn()
        self.execute_straight_drive()
            
        # STEP 5: RESUME autonomy
        if not self._call_set_bool_service(self.resume_client, True, 'RESUME'):
            self.get_logger().error("RESUME failed. Robot might remain paused.")
            return
        
        # STEP 6: BLOCK and wait for GoToPOI to finish
        self.wait_for_go_to_poi_completion(poi_future)
        
        self.get_logger().info("Mission execution complete.")


def main(args=None):
    rclpy.init(args=args)
    
    planner = MissionPlanner()
    
    try:
        planner.execute_mission()
    except Exception as e:
        planner.get_logger().error(f"Mission execution failed with an unhandled exception: {e}")
    finally:
        
        planner._call_trigger_service(planner.autonomy_stop_client, 'AUTONOMY STOP (Shutdown)') 
        
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
