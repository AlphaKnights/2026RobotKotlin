/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.pathplanner.lib.commands.FollowPathCommand
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.*
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.Constants.AimingConstants
import frc.robot.Constants.DriveConstants
import frc.robot.subsystems.DriveSubsystem.drive

object AimingPathPlanner {
    fun generatePath(): Pose2d {
        val curpose = DriveSubsystem.getPose()

        val hubPos = Translation2d(AimingConstants.BLUE_HUB_X, AimingConstants.BLUE_HUB_Y)
        val distanceHub = curpose.translation - hubPos // distance between robot and hub
        val scalar = AimingConstants.DISTANCE / distanceHub.norm // creates a scalar to find a position at the right distance and direction from the hub (hub relative)
        val target = hubPos + (distanceHub * scalar) // finds the field relative position of the scaled vector

        return Pose2d(target, curpose.rotation)

        //        val constraints =
//            PathConstraints(
//                DriveConstants.MAX_METERS_PER_SECOND,
//                10.0,
//                DriveConstants.MAX_ANGULAR_SPEED,
//                4 * Math.PI,
//            )
//        val endState = GoalEndState(0.0, curpose.rotation)
//        val waypoints =
//            PathPlannerPath.waypointsFromPoses(
//                curpose,
//                Pose2d(target, curpose.rotation)
//            )

//        return PathPlannerPath(
//            waypoints,
//            mutableListOf<RotationTarget>(),
//            mutableListOf<PointTowardsZone>(
//                PointTowardsZone(
//                    "Hub",
//                    hubPos,
//                    0.0,
//                    1.0,
//                ),
//            ),
//            mutableListOf<ConstraintsZone>(),
//            mutableListOf<EventMarker>(),
//            constraints,
//            null,
//            endState,
//            false,
//        )
    }
}
