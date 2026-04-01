package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DriveSubsystem

class ResetOdometry : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        DriveSubsystem.resetOdometry(Pose2d(0.0,0.0, Rotation2d(0.0) )) // DriveSubsystem.getPose().rotation
    }
}