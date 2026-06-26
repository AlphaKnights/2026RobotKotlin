/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DriveSubsystem

class LockXCommand : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        DriveSubsystem.setX()
    }
}
