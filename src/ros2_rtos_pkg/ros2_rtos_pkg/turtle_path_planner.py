import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtlePathPlanner(Node):

    def __init__(self):
        super().__init__('turtle_path_planner')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        self.goal_x = 40.0
        self.goal_y = 40.0
        self.obstacle_min_x = 30.0
        self.obstacle_min_y = 30.0
        self.obstacle_max_x = 35.0
        self.obstacle_max_y = 35.0

        # Get user input for start position and orientation
        self.start_x = float(input("Enter start x position (between 0 and 40): "))
        self.start_y = float(input("Enter start y position (between 0 and 40): "))
        self.heading = float(input("Enter heading orientation (in radians, 0 to 2*pi): "))

    def pose_callback(self, msg):
        self.pose = msg

    def move_to_goal(self):
        rate = self.create_rate(10)  # 10 Hz loop rate

        # Check if pose is received before moving
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self)

        while rclpy.ok():
            if self.pose is None:
                continue

            # Calculate distance and heading to goal
            distance = math.sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2)
            heading_angle = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)
            heading_error = heading_angle - self.pose.theta

            # If goal reached, stop
            if distance < 0.5:
                print("Goal reached!")
                break

            # Obstacle avoidance logic
            if (self.obstacle_min_x <= self.pose.x <= self.obstacle_max_x and
                self.obstacle_min_y <= self.pose.y <= self.obstacle_max_y):
                print("Obstacle detected, adjusting path!")
                heading_error += math.pi / 4  # Change heading to avoid obstacle

            # Proportional control for speed and heading
            linear_velocity = 0.5 * distance
            angular_velocity = 2.0 * heading_error

            # Limit velocities to prevent excessive movement
            linear_velocity = min(linear_velocity, 1.0)
            angular_velocity = max(min(angular_velocity, 1.0), -1.0)

            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity

            self.publisher_.publish(twist)
            rclpy.spin_once(self)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    turtle_planner = TurtlePathPlanner()
    turtle_planner.move_to_goal()
    turtle_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

