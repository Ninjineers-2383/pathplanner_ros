import math
import threading

import rclpy.qos
import rclpy.time
from .path import PathPlannerPath
from .config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
from .telemetry import PPLibTelemetry
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from pathplanner_ros_interfaces.srv import PathPlannerFollowerCreate
from pathplanner_ros_interfaces.action import FollowPath
from .commands import FollowPathHolonomic
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, TwistStamped

from std_msgs.msg import String

class PathFollowedAction:
    def __init__(self, node: Node, name: str, command: FollowPathHolonomic):
        self.node = node
        self.command = command
        self.server = ActionServer(node, FollowPath, name,
                                   execute_callback=self.execute,
                                   handle_accepted_callback=self.handle_accepted_callback,
                                   goal_callback=self.goal_callback,
                                   cancel_callback=self.cancel_callback
                                   )
        self._goal_handle = None
        self._goal_lock = threading.Lock()

    def goal_callback(self, goal_handle: ServerGoalHandle):
        self.node.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal: ServerGoalHandle):
        """Accept or reject a client request to cancel an action."""
        self.node.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.node.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        self.command.initialize()
        goal_handle.execute()

    def execute(self, goal_handle: ServerGoalHandle):
        rate = self.node.create_rate(100, self.node.get_clock())

        while rclpy.ok() and goal_handle.is_active and not self.command.isFinished():
            self.command.execute()
            rate.sleep()

        self.command.end(not goal_handle.is_active)

        goal_handle.succeed()
        result = FollowPath.Result()
        return result


class PathPlannerAutoServer(Node):
    def __init__(self):
        super().__init__('pathplanner_auto_creation_service')
        self.get_logger().info("Starting creation server")
        self.srv = self.create_service(PathPlannerFollowerCreate, 'create_pathplanner_follower', self.create_follower_callback)
        self.actions = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/swerve_drive_controller/cmd_vel', rclpy.qos.qos_profile_system_default)

    def create_follower_callback(self, request: PathPlannerFollowerCreate.Request, response: PathPlannerFollowerCreate.Response):
        self.get_logger().info(f"Creating follower for {request}")
        path = PathPlannerPath.fromPathFile(f"{request.package}:{request.pathname}")
        config = HolonomicPathFollowerConfig(
            PIDConstants(5, 0, 0),
            PIDConstants(10, 0, 0),
            4,
            0.5,
            ReplanningConfig(False, False, float('inf'), float('inf'))
        )
        command = FollowPathHolonomic(path, self.get_robot_pose, lambda : ChassisSpeeds(), self.post_chassis_speed, config, lambda : False, self.get_clock())
        self.actions.append(PathFollowedAction(self, f"{request.package}_{request.pathname}_follow", command))
        response.follow_action = f"{request.package}_{request.pathname}_follow"

        return response
    
    def get_robot_pose(self) -> Pose2d:
        if self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
            tf: TransformStamped = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            pose = Pose2d(tf.transform.translation.x, tf.transform.translation.y, Rotation2d(tf.transform.rotation.z))
            self.get_logger().info(f'Robot Pose: {pose}')
            return pose
        else:
            self.get_logger().error('Culd not lookup robot pose')
            return Pose2d()
        
    def post_chassis_speed(self, speeds: ChassisSpeeds):
        message = TwistStamped()
        message.header.frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x = speeds.vx
        message.twist.linear.y = speeds.vy
        message.twist.angular.z = speeds.omega * math.pi / 180.0
        self.cmd_vel_pub.publish(message)
        self.get_logger().info(f'Moving {speeds}')

def main(args=None):
    rclpy.init(args=args)

    root_node = PathPlannerAutoServer()

    PPLibTelemetry.initialize(root_node)

    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(root_node)
    try:
        rclpy.spin(root_node, multi_thread_executor)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()