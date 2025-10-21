import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 
from rclpy.task import Future
from geometry_msgs.msg import Twist, TwistStamped
import time

# ==============================================================================
# 1. CORE ROS MESSAGE IMPORTS
# ==============================================================================
try:
    # Services: SetBool for pause/resume, Trigger for autonomy stop
    from std_srvs.srv import SetBool, Trigger 
    # Actions: GoToPOI
    from clearpath_navigation_msgs.action import ExecuteNetworkGoToPOI as GoToPoi 
except ImportError as e:
    # This check helps quickly diagnose issues if the environment isn't sourced.
    print(f"FATAL: Missing ROS 2 messages. Did you forget to source? Error: {e}")
    raise


# ==============================================================================
# 2. MISSION PLANNER NODE
# ==============================================================================

class MissionPlanner(Node):
    """
    Executes a sequential mission designed for a planned interruption:
    1. Start GoToPOI (non-blocking, captures future).
    2. Wait 5 seconds to interrupt navigation mid-route.
    3. Pause the control stack.
    4. Execute blocking local maneuvers (Turn and Drive via cmd_vel).
    5. Resume control.
    6. BLOCK and wait for GoToPOI to finally complete (the "spin until complete" part).
    
    The final Autonomy Stop command is issued during program shutdown.
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
        
        # Setup Clients
        self.poi_client = ActionClient(self, GoToPoi, self.ACTION_GO_TO_POI)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, self.CMD_VEL_TOPIC, 10)
        self.pause_client = self.create_client(SetBool, self.SERVICE_PAUSE)
        self.resume_client = self.create_client(SetBool, self.SERVICE_RESUME)
        self.autonomy_stop_client = self.create_client(Trigger, self.SERVICE_STOP)
        
        self._wait_for_clients()
        
    def _wait_for_clients(self):
        """Blocks until all necessary ROS servers are available."""
        self.get_logger().info('Pinging mission components...')
        
        while not all([
            self.poi_client.wait_for_server(timeout_sec=1.0),
            self.pause_client.wait_for_service(timeout_sec=1.0),
            self.resume_client.wait_for_service(timeout_sec=1.0),
            self.autonomy_stop_client.wait_for_service(timeout_sec=1.0),
        ]):
            self.get_logger().info('Still waiting...')
            time.sleep(1.0)
            
        self.get_logger().info('All components online. Ready.')

    # --- TWIST PUBLISHING UTILITY ---------------------------------------------

    def publish_twist(self, linear_x, angular_z):
        """Publishes a TwistStamped message for direct motion control."""
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = twist
        
        self.cmd_vel_publisher.publish(twist_stamped)

    # --- TELEOP MOTION WRAPPERS (Blocking until time expires) -----------------

    def execute_teleop_motion(self, linear_vel, angular_vel, duration, action_name):
        """Performs approximate motion via fixed-duration publishing and blocks."""
        self.get_logger().info(f'--- MOTION START (BLOCKING): {action_name} for {duration:.2f}s ---')
        
        start_time_s = time.time()
        self.publish_twist(linear_vel, angular_vel)
        
        # Loop to ensure publishing continues
        end_wait_time = time.time() + duration
        while time.time() < end_wait_time:
            self.publish_twist(linear_vel, angular_vel)
            time.sleep(self.PUBLISH_RATE)
        
        self.publish_twist(0.0, 0.0) # Stop motion
        end_time_s = time.time()
        
        self.get_logger().info(f'SUCCESS: {action_name} finished (Actual duration: {end_time_s - start_time_s:.2f}s).')
        return True
        
    def execute_turn(self):
        """Executes the approximate 180-degree turn."""
        return self.execute_teleop_motion(0.0, self.TURN_VEL_RAD_S, self.TURN_DURATION, f'180 DEGREE TURN')

    def execute_straight_drive(self):
        """Executes the approximate 1-meter drive."""
        return self.execute_teleop_motion(self.DRIVE_VEL_M_S, 0.0, self.DRIVE_DURATION, f'1 METER DRIVE')

    # --- ACTION WRAPPER (Non-Blocking) ----------------------------------------

    def send_go_to_poi_goal(self, poi_uuid: str, map_uuid: str):
        """Sends GoToPOI goal and returns the future for later blocking."""
        self.get_logger().info(f'--- NAV START (NON-BLOCKING): Go to POI "{poi_uuid}" ---')
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


    # --- SERVICE WRAPPERS -----------------------------------------------------

    def _call_set_bool_service(self, client, data: bool, action_name: str):
        """Helper for calling SetBool services (Pause/Resume)."""
        self.get_logger().info(f'Attempting to call {action_name} service...')
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

    # --- MISSION EXECUTION ----------------------------------------------------

    def execute_mission(self):
        """The main, sequential mission logic."""
        self.get_logger().info("\n--- MISSION START ---")
        
        # STEP 1: Initiate GoToPOI (Fire and Forget, capture future)
        poi_future = self.send_go_to_poi_goal(self.POI_ID, self.MAP_ID)
        
        # STEP 2: Wait 5 seconds to interrupt navigation mid-route
        self.get_logger().info('PRE-PAUSE DELAY: Sleeping for 5 seconds while robot drives...')
        time.sleep(5)
        self.get_logger().info('Waking up. Initiating pause.')
        
        # STEP 3: PAUSE control selection
        if not self._call_set_bool_service(self.pause_client, True, 'PAUSE'):
            self.get_logger().error("PAUSE failed. Aborting mission.")
            return

        # STEP 4: Perform local maneuvers (Blocking, executed while paused)
        # Note: The 'spin' (execute_turn) is performed here.
        self.execute_turn()
        self.execute_straight_drive()
            
        # STEP 5: RESUME control selection (Robot should continue GoToPOI)
        if not self._call_set_bool_service(self.resume_client, True, 'RESUME'):
            self.get_logger().error("RESUME failed. Robot might remain paused.")
            return
        
        # STEP 6: BLOCK and wait for GoToPOI to finish (The final 'spin until complete')
        self.wait_for_go_to_poi_completion(poi_future)
        
        self.get_logger().info("\n--- MISSION COMPLETE (POI reached and sequence finished) ---")


def main(args=None):
    rclpy.init(args=args)
    
    planner = MissionPlanner()
    
    try:
        planner.execute_mission()
    except Exception as e:
        planner.get_logger().error(f"Mission execution failed with an unhandled exception: {e}")
    finally:
        # Final, system-level stop on shutdown (explicitly requested)
        planner.get_logger().info("Mission sequence finished. Triggering Autonomy Stop on shutdown.")
        planner._call_trigger_service(planner.autonomy_stop_client, 'AUTONOMY STOP (Shutdown)') 
        
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
