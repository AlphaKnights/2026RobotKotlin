/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.autoalign

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.AutoAlignCalc
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem

class AutoAlignAutoCommand(
    private val direction: Constants.AlignDirection,
) : Command() {
    private val timer = Timer()

    init {
        addRequirements(DriveSubsystem)
    }

    override fun initialize() {
        super.initialize()
        timer.start()
        timer.reset()
    }

    override fun execute() {
        val currentPose = LimelightSubsystem.tagPose
        if (currentPose == null) {
            seekTagByRotating()
            return
        }
        timer.reset()
        driveToAlignmentPosition(currentPose)
    }

    // If we haven't seen a tag for ALIGN_SEEK_TIMEOUT seconds, slowly rotate to scan
    // for one. We wait the timeout first so a momentary vision dropout doesn't
    // immediately trigger a spin.
    private fun seekTagByRotating() {
        if (timer.get() > Constants.AlignConstants.ALIGN_SEEK_TIMEOUT) {
            DriveSubsystem.drive(
                // Bug fix: was Constants.AlignConstants.MAX_SPEED (meters/second) —
                // a linear speed constant used as an angular speed. MAX_SPEED and
                // MAX_ANGULAR_SPEED currently share the value 1.0 so this was invisible,
                // but they measure different physical quantities. Units matter: if either
                // constant is ever tuned independently, this would silently produce the
                // wrong rotation rate.
                ChassisSpeeds(0.0, 0.0, Constants.AlignConstants.MAX_ANGULAR_SPEED),
                fieldRelative = false,
            )
        }
    }

    private fun driveToAlignmentPosition(currentPose: Pose3d) {
        val (goalX, goalZ) = goalOffsetForDirection()
        val speeds = AutoAlignCalc.getAlignSpeeds(goalX, goalZ, currentPose)
        if (speeds.vxMetersPerSecond == 0.0 &&
            speeds.vyMetersPerSecond == 0.0 &&
            speeds.omegaRadiansPerSecond == 0.0
        ) {
            DriveSubsystem.setX()
        } else {
            DriveSubsystem.drive(speeds, fieldRelative = false)
        }
    }

    private fun goalOffsetForDirection(): Pair<Double, Double> =
        when (direction) {
            Constants.AlignDirection.LEFT ->
                Pair(Constants.AlignConstants.LEFT_X_OFFSET, Constants.AlignConstants.LEFT_Z_OFFSET)
            Constants.AlignDirection.RIGHT ->
                Pair(Constants.AlignConstants.RIGHT_X_OFFSET, Constants.AlignConstants.RIGHT_Z_OFFSET)
        }

    // Bug fix: The original isFinished() contained DriveSubsystem.drive() calls.
    // WPILib's Command framework treats isFinished() as a simple yes/no question:
    // "Are we done yet?" It must NOT move the robot or produce any other side effects.
    // The scheduler can call isFinished() at unexpected times, and putting drive logic
    // here leads to unpredictable robot behavior. All driving now lives in execute().
    override fun isFinished(): Boolean =
        timer.get() > Constants.AlignConstants.ALIGN_TIMEOUT ||
            LimelightSubsystem.isAligned(direction)
}
