package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class PathfindToPoseCommand(pose: Pose2d) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    val pose = pose

    override fun execute() {
        Commands.sequence(
            AutoBuilder.pathfindToPose(
                pose,
                Constants.DriveConstants.PATH_CONSTRAINTS,
            ),
        )
    }
}
