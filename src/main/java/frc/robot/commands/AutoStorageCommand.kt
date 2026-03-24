package frc.robot.commands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.Constants
import frc.robot.subsystems.StorageSubsystem

class AutoStorageCommand(reversed: Boolean) : Command() {

    val reverse = reversed
    val timer: Timer = Timer()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(StorageSubsystem)
    }

    override fun initialize() {
        super.initialize()
        timer.start()
    }

    override fun execute() {
        if (reverse) {
            StorageSubsystem.roll(-Constants.RollerConstants.ROLLER_SPEED)
        }
        else {
            StorageSubsystem.roll(Constants.RollerConstants.ROLLER_SPEED)
        }
    }

    override fun isFinished(): Boolean {
        return if (timer.get() > 3) true else false
    }

    override fun end(interrupted: Boolean) {
        StorageSubsystem.rollerstop()
        timer.stop()
        timer.reset()
        super.end(interrupted)
    }
}
