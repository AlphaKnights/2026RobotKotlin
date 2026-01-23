/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DriveSubsystem

class ResetHeadingCommand : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        DriveSubsystem.zeroHeading()
    }

    override fun isFinished(): Boolean = false
}
