/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.autoalign

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.AutoAlignCalc
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem

class AutoAlignManualCommand(
    private val direction: Constants.AlignDirection,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        val curPose =
            LimelightSubsystem.tagPose ?: run {
                DriveSubsystem.setX()
                return
            }

        val (goalX, goalZ) = goalOffsetForDirection()
        val speeds = AutoAlignCalc.getAlignSpeeds(goalX, goalZ, curPose)

        if (
            speeds.vxMetersPerSecond == 0.0 &&
            speeds.vyMetersPerSecond == 0.0 &&
            speeds.omegaRadiansPerSecond == 0.0
        ) {
            DriveSubsystem.setX()
            return
        }

        DriveSubsystem.drive(
            speeds,
            fieldRelative = false,
        )
    }

    private fun goalOffsetForDirection(): Pair<Double, Double> =
        when (direction) {
            Constants.AlignDirection.LEFT ->
                Pair(Constants.AlignConstants.LEFT_X_OFFSET, Constants.AlignConstants.LEFT_Z_OFFSET)
            Constants.AlignDirection.RIGHT ->
                Pair(Constants.AlignConstants.RIGHT_X_OFFSET, Constants.AlignConstants.RIGHT_Z_OFFSET)
        }

    override fun isFinished(): Boolean = false
}
