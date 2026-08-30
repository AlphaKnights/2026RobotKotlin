package frc.robot.commands

import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command3.Command

class LockXCommand : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        DriveSubsystem.setX()
    }
}
