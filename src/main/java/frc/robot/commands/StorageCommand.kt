package frc.robot.commands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants
import frc.robot.subsystems.StorageSubsystem
import frc.robot.Constants.StorageConstants

class StorageCommand () : Command() {
    init {
        addRequirements(StorageSubsystem)
    }

    override fun execute() {
        StorageSubsystem.runStorage(StorageConstants.MOTOR_SPEED)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        StorageSubsystem.stopStorage()
    }
}
