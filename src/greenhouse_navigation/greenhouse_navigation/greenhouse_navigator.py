#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import time

class GreenhouseNavigator(Node):
    def __init__(self):
        super().__init__('greenhouse_navigator')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigation action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Navigation server available!')
        
        # Define the waypoints for our greenhouse path (these coordinates need to be adjusted for your map)
        # Format: [x, y, orientation_z, orientation_w, description]
        self.waypoints = [
            # Row 1 start
            [1.0, 0.0, 0.0, 1.0, "Row 1 Start"],
            # Row 1 end - stop here for 3 seconds
            [5.0, 0.0, 0.0, 1.0, "Row 1 End - Stop Point"],
            # Turn to Row 2
            [5.0, 1.0, 0.7071, 0.7071, "Turn to Row 2"],
            # Row 2 start
            [5.0, 2.0, 1.0, 0.0, "Row 2 Start"],
            # Row 2 end - stop here for 3 seconds
            [1.0, 2.0, 1.0, 0.0, "Row 2 End - Stop Point"],
            # Turn to Row 3
            [1.0, 3.0, 0.7071, 0.7071, "Turn to Row 3"],
            # Row 3 start
            [1.0, 4.0, 0.0, 1.0, "Row 3 Start"],
            # Row 3 end - stop here for 3 seconds
            [5.0, 4.0, 0.0, 1.0, "Row 3 End - Stop Point"],
            # Return to start
            [1.0, 0.0, 0.0, 1.0, "Return to Start"],
        ]
        
        # Flag to indicate stopping points
        self.stop_points = [1, 4, 7]  # Indices of waypoints where robot should stop for 3 sec
        
        self.current_waypoint = 0
        self.goal_handle = None
        
        # Start navigation
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Navigation complete! All waypoints visited.')
            return
        
        waypoint = self.waypoints[self.current_waypoint]
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint}: {waypoint[4]}')
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint[0]
        goal_msg.pose.pose.position.y = waypoint[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = waypoint[2]
        goal_msg.pose.pose.orientation.w = waypoint[3]
        
        # Send goal
        self.goal_handle = self.nav_client.send_goal_async(goal_msg)
        self.goal_handle.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached waypoint {self.current_waypoint}')
            
            # Check if we should stop here
            if self.current_waypoint in self.stop_points:
                self.get_logger().info(f'Stopping for 3 seconds at {self.waypoints[self.current_waypoint][4]}')
                time.sleep(3.0)  # Stop for 3 seconds
                self.get_logger().info('Continuing to next waypoint')
            
            # Move to next waypoint
            self.current_waypoint += 1
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error(f'Goal failed with status: {status}')

def main(args=None):
    rclpy.init(args=args)
    navigator = GreenhouseNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation canceled by user')
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()