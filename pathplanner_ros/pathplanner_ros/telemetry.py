from wpimath.geometry import Pose2d

import rclpy.qos
from .path import PathPlannerPath
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from wpimath.geometry import Rotation3d

class PPLibTelemetry:
    # _velPub: DoubleArrayPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic('/PathPlanner/vel').publish()
    # _inaccuracyPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic(
    #     '/PathPlanner/inaccuracy').publish()
    # _posePub: DoubleArrayPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(
    #     '/PathPlanner/currentPose').publish()
    # _pathPub: DoubleArrayPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(
    #     '/PathPlanner/activePath').publish()
    # _targetPosePub: DoubleArrayPublisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(
    #     '/PathPlanner/targetPose').publish()
    @classmethod
    def initialize(cls, node: Node):
        cls._clock = node.get_clock()
        cls._node = node
        cls._targetPosePub = tf2_ros.TransformBroadcaster(node)

    @staticmethod
    def setVelocities(actual_vel: float, commanded_vel: float, actual_ang_vel: float, commanded_ang_vel: float) -> None:
        pass
        #PPLibTelemetry._velPub.set([actual_vel, commanded_vel, actual_ang_vel, commanded_ang_vel])

    @staticmethod
    def setPathInaccuracy(inaccuracy: float) -> None:
        pass
        #PPLibTelemetry._inaccuracyPub.set(inaccuracy)

    @staticmethod
    def setCurrentPose(pose: Pose2d) -> None:
        pass
        #PPLibTelemetry._posePub.set([pose.X(), pose.Y(), pose.rotation().radians()])

    @staticmethod
    def setTargetPose(pose: Pose2d) -> None:
        tf = TransformStamped()
        quat = Rotation3d(0.0, 0.0, pose.rotation().radians()).getQuaternion()
        tf.header.stamp = PPLibTelemetry._clock.now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'ghost/base_link'
        tf.transform.translation.x = pose.x
        tf.transform.translation.y = pose.y
        tf.transform.rotation.x = quat.X()
        tf.transform.rotation.y = quat.Y()
        tf.transform.rotation.z = quat.Z()
        tf.transform.rotation.w = quat.W()
        PPLibTelemetry._targetPosePub.sendTransform(tf)
        PPLibTelemetry._node.get_logger().info(f'Target pose: {pose}, {tf}')
        pass
        #PPLibTelemetry._targetPosePub.set([pose.X(), pose.Y(), pose.rotation().radians()])

    @staticmethod
    def setCurrentPath(path: PathPlannerPath) -> None:
        # arr = []

        # for p in path.getAllPathPoints():
        #     arr.extend([p.position.X(), p.position.Y(), 0.0])

        # PPLibTelemetry._pathPub.set(arr)
        pass
