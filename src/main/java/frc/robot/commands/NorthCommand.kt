package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class NorthCommand: Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()

        DriveSubsystem.drive(
            ChassisSpeeds(
                0.0,
                0.0,
                -DriveSubsystem.getPose().rotation.radians,
            ),
            fieldRelative = false,
        )
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {

    }
}
