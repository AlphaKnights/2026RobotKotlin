/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.StorageSubsystem

class StorageCommand(
    reversed: Boolean,
) : Command() {
    val reverse = reversed

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(StorageSubsystem)
    }

    override fun execute() {
        if (reverse) {
            StorageSubsystem.roll(-Constants.RollerConstants.ROLLER_SPEED)
        } else {
            StorageSubsystem.roll(Constants.RollerConstants.ROLLER_SPEED)
        }
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        StorageSubsystem.rollerstop()
        super.end(interrupted)
    }
}
