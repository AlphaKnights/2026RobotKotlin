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

    override fun initialize() {
        super.initialize()
    }

    override fun execute() {
        IntakeSubsystem.setPosition(targetPosition)
    }

    override fun isFinished(): Boolean {
        return IntakeSubsystem.limitSwitchPressed()
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopIntakeLever()

    }

}