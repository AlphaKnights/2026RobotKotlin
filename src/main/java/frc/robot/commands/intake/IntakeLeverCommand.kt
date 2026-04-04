package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.subsystems.IntakeSubsystem

class IntakeLeverCommand(
    private val targetPosition: Double
) : Command(){

    init {
        addRequirements(IntakeSubsystem)
    }

    override fun execute() {
        IntakeSubsystem.setPosition(targetPosition)
    }

    override fun isFinished(): Boolean {
        return IntakeSubsystem.limitSwitchPressed() or IntakeSubsystem.isInPosition(0.1)
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopIntakeLever()

    }

}