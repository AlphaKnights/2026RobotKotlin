package frc.robot.commands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants
import frc.robot.subsystems.StorageSubsystem

class StorageCommand : Command() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(StorageSubsystem)

    }

    override fun execute() {
        StorageSubsystem.roll(Constants.RollerConstants.ROLLER_SPEED)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        StorageSubsystem.rollerstop()
        super.end(interrupted)
    }
}
