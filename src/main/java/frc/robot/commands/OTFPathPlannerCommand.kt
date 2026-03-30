package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.subsystems.AimingPathPlanner
import frc.robot.subsystems.DriveSubsystem

class OTFPathPlannerCommand : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        Commands.sequence(
            AutoBuilder.pathfindToPose(
                AimingPathPlanner.generatePath(),
                Constants.DriveConstants.PATH_CONSTRAINTS
            ),
        )
    }
}
