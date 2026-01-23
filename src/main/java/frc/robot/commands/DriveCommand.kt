/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class DriveCommand(
    private val x: () -> Double,
    private val y: () -> Double,
    private val rot: () -> Double,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()

        DriveSubsystem.drive(
            ChassisSpeeds(
                x() *
                    Constants.DriveConstants.MAX_METERS_PER_SECOND,
                y() *
                    Constants.DriveConstants.MAX_METERS_PER_SECOND,
                rot() *
                    Constants.DriveConstants.MAX_ANGULAR_SPEED,
            ),
            fieldRelative = true,
        )
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.drive(
            ChassisSpeeds(
                0.0,
                0.0,
                0.0,
            ),
            fieldRelative = false,
        )
    }
}
