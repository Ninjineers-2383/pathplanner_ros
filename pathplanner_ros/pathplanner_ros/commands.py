import math

import rclpy.clock

from .command import Command

from .controller import *
from .path import PathPlannerPath, EventMarker, GoalEndState, PathConstraints
from .trajectory import PathPlannerTrajectory
from .telemetry import PPLibTelemetry
from .logging import PathPlannerLogging
from .geometry_util import floatLerp, flipFieldPose
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from typing import Callable, Tuple, List
from .config import ReplanningConfig, HolonomicPathFollowerConfig
from .pathfinding import Pathfinding

import rclpy

class FollowPathCommand:
    _originalPath: PathPlannerPath
    _poseSupplier: Callable[[], Pose2d]
    _speedsSupplier: Callable[[], ChassisSpeeds]
    _output: Callable[[ChassisSpeeds], None]
    _controller: PathFollowingController
    _replanningConfig: ReplanningConfig
    _shouldFlipPath: Callable[[], bool]

    # For event markers
    _currentEventCommands: dict = {}
    _untriggeredEvents: List[Tuple[float, Command]] = []

    _timer: rclpy.clock.Clock
    _path: PathPlannerPath = None
    _generatedTrajectory: PathPlannerTrajectory = None

    _start_time: rclpy.clock.Time = None

    def __init__(self, path: PathPlannerPath, pose_supplier: Callable[[], Pose2d],
                 speeds_supplier: Callable[[], ChassisSpeeds], output_robot_relative: Callable[[ChassisSpeeds], None],
                 controller: PathFollowingController, replanning_config: ReplanningConfig,
                 should_flip_path: Callable[[], bool], clock: rclpy.clock.Clock):
        """
        Construct a base path following command

        :param path: The path to follow
        :param pose_supplier: Function that supplies the current field-relative pose of the robot
        :param speeds_supplier: Function that supplies the current robot-relative chassis speeds
        :param output_robot_relative: Function that will apply the robot-relative output speeds of this command
        :param controller: Path following controller that will be used to follow the path
        :param replanning_config: Path replanning configuration
        :param should_flip_path: Should the path be flipped to the other side of the field? This will maintain a global blue alliance origin.
        :param requirements: Subsystems required by this command, usually just the drive subsystem
        """
        super().__init__()

        self._originalPath = path
        self._poseSupplier = pose_supplier
        self._speedsSupplier = speeds_supplier
        self._output = output_robot_relative
        self._controller = controller
        self._replanningConfig = replanning_config
        self._shouldFlipPath = should_flip_path
        self._timer = clock

    def initialize(self):
        if self._shouldFlipPath() and not self._originalPath.preventFlipping:
            self._path = self._originalPath.flipPath()
        else:
            self._path = self._originalPath

        currentPose = self._poseSupplier()
        currentSpeeds = self._speedsSupplier()

        self._controller.reset(currentPose, currentSpeeds)

        fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.rotation())
        currentHeading = Rotation2d(fieldSpeeds.vx, fieldSpeeds.vy)
        targetHeading = (self._path.getPoint(1).position - self._path.getPoint(0).position).angle()
        headingError = currentHeading - targetHeading
        onHeading = math.hypot(currentSpeeds.vx, currentSpeeds.vy) < 0.25 or abs(headingError.degrees()) < 30

        if not self._path.isChoreoPath() and self._replanningConfig.enableInitialReplanning and (
                currentPose.translation().distance(self._path.getPoint(0).position) > 0.25 or not onHeading):
            self._replanPath(currentPose, currentSpeeds)
        else:
            self._generatedTrajectory = self._path.getTrajectory(currentSpeeds, currentPose.rotation())
            PathPlannerLogging.logActivePath(self._path)
            PPLibTelemetry.setCurrentPath(self._path)

        # Initialize marker stuff
        self._currentEventCommands.clear()
        self._untriggeredEvents.clear()
        for event in self._generatedTrajectory.getEventCommands():
            self._untriggeredEvents.append(event)

        self._start_time = self._timer.now()

    def execute(self):
        currentTime = (self._timer.now().nanoseconds / 1E9) - (self._start_time.nanoseconds / 1E9)
        targetState = self._generatedTrajectory.sample(currentTime)
        if not self._controller.isHolonomic() and self._path.isReversed():
            targetState = targetState.reverse()

        PPLibTelemetry.setCurrentTime(currentTime)

        currentPose = self._poseSupplier()
        currentSpeeds = self._speedsSupplier()

        if not self._path.isChoreoPath() and self._replanningConfig.enableDynamicReplanning:
            previousError = abs(self._controller.getPositionalError())
            currentError = currentPose.translation().distance(targetState.positionMeters)

            if currentError >= self._replanningConfig.dynamicReplanningTotalErrorThreshold or currentError - previousError >= self._replanningConfig.dynamicReplanningErrorSpikeThreshold:
                self._replanPath(currentPose, currentSpeeds)
                self._start_time = self._timer.now()
                targetState = self._generatedTrajectory.sample(0.0)

        targetSpeeds = self._controller.calculateRobotRelativeSpeeds(currentPose, targetState)

        currentVel = math.hypot(currentSpeeds.vx, currentSpeeds.vy)

        PPLibTelemetry.setCurrentPose(currentPose)
        PathPlannerLogging.logCurrentPose(currentPose)

        if self._controller.isHolonomic():
            PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose())
            PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose())
        else:
            PPLibTelemetry.setTargetPose(targetState.getDifferentialPose())
            PathPlannerLogging.logTargetPose(targetState.getDifferentialPose())

        PPLibTelemetry.setVelocities(currentVel, targetState.velocityMps, currentSpeeds.omega, targetSpeeds.omega)
        PPLibTelemetry.setPathInaccuracy(self._controller.getPositionalError())

        self._output(targetSpeeds)

        if len(self._untriggeredEvents) > 0 and (self._timer.now().seconds_nanoseconds()[0] - self._start_time.seconds_nanoseconds()[0]) >=self._untriggeredEvents[0][0]:
            # Time to trigger this event command
            cmd = self._untriggeredEvents.pop(0)[1]

            for command in self._currentEventCommands:
                if not self._currentEventCommands[command]:
                    continue

                for req in command.getRequirements():
                    if req in cmd.getRequirements():
                        command.end(True)
                        self._currentEventCommands[command] = False
                        break

            cmd.initialize()
            self._currentEventCommands[cmd] = True

        # Execute event marker commands
        for command in self._currentEventCommands:
            if not self._currentEventCommands[command]:
                continue

            command.execute()

            if command.isFinished():
                command.end(False)
                self._currentEventCommands[command] = False

    def isFinished(self) -> bool:
        if (self._start_time is None):
            return False
        return (self._timer.now().nanoseconds - self._start_time.nanoseconds) >= self._generatedTrajectory.getTotalTimeSeconds() * 1E9

    def end(self, interrupted: bool):

        # Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
        # the command to smoothly transition into some auto-alignment routine
        if not interrupted and self._path.getGoalEndState().velocity < 0.1:
            self._output(ChassisSpeeds())

        PathPlannerLogging.logActivePath(None)

        # End event marker commands
        for command in self._currentEventCommands:
            if self._currentEventCommands[command]:
                command.end(True)

    def _replanPath(self, current_pose: Pose2d, current_speeds: ChassisSpeeds) -> None:
        replanned = self._path.replan(current_pose, current_speeds)
        self._generatedTrajectory = replanned.getTrajectory(current_speeds, current_pose.rotation())
        PathPlannerLogging.logActivePath(replanned)
        PPLibTelemetry.setCurrentPath(replanned)


class FollowPathHolonomic(FollowPathCommand):
    def __init__(self, path: PathPlannerPath, pose_supplier: Callable[[], Pose2d],
                 speeds_supplier: Callable[[], ChassisSpeeds], output_robot_relative: Callable[[ChassisSpeeds], None],
                 config: HolonomicPathFollowerConfig, should_flip_path: Callable[[], bool], clock: rclpy.clock.Clock):
        """
        Construct a path following command that will use a holonomic drive controller for holonomic drive trains

        :param path: The path to follow
        :param pose_supplier: Function that supplies the current field-relative pose of the robot
        :param speeds_supplier: Function that supplies the current robot-relative chassis speeds
        :param output_robot_relative: Function that will apply the robot-relative output speeds of this command
        :param config: Holonomic path follower configuration
        :param should_flip_path: Should the path be flipped to the other side of the field? This will maintain a global blue alliance origin.
        :param requirements: Subsystems required by this command, usually just the drive subsystem
        """
        super().__init__(path, pose_supplier, speeds_supplier, output_robot_relative, PPHolonomicDriveController(
            config.translationConstants, config.rotationConstants, config.maxModuleSpeed, config.driveBaseRadius,
            config.period
        ), config.replanningConfig, should_flip_path, clock)

