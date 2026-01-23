package frc.robot.commands.coralmanipulator

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.CoralManipulatorSubsystem

class LaunchCommand : Command() {
    private val timer = Timer()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(CoralManipulatorSubsystem)

        timer.start()
    }

    override fun initialize() {
        super.initialize()
        timer.reset()
    }

    override fun execute() {
        CoralManipulatorSubsystem.forward(1.0)
    }

    override fun isFinished(): Boolean {
        // Run until the coral is out of the robot
        return (
                !CoralManipulatorSubsystem.coralInside() ||
                timer.get() > 3 // seconds
                )
    }

    override fun end(interrupted: Boolean) {
        CoralManipulatorSubsystem.stop()
        super.end(interrupted)
    }
}
