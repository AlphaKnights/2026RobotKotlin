package frc.robot.commands.intake

import frc.robot.subsystems.IntakeSubsystem
import org.wpilib.command3.Command
import org.wpilib.wpilibj.DigitalInput

class IntakeLeverCommand(
    private val targetPosition: Double,
) : Command() {
    init {
        addRequirements(IntakeSubsystem)
    }

    override fun execute() {
        IntakeSubsystem.setPosition(targetPosition)
    }

    override fun isFinished(): Boolean = IntakeSubsystem.limitSwitchPressed() or IntakeSubsystem.isInPosition(1.0)

    override fun end(interrupted: Boolean) {
        // IntakeSubsystem.stopIntakeLever()
    }
}
