package frc.robot.commands.coralmanipulator

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.CoralManipulatorSubsystem

class IntakeCommand(bool: Boolean) : Command() {
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
        print(CoralManipulatorSubsystem.coralInside())
        CoralManipulatorSubsystem.forward(0.3)

    }

    override fun isFinished(): Boolean {
        // Run until the coral is inside the robot
        return (
                CoralManipulatorSubsystem.coralInside() ||
                timer.get() > 10 // seconds
                )
    }

    override fun end(interrupted: Boolean) {
        CoralManipulatorSubsystem.stop()
        super.end(interrupted)
    }
}
