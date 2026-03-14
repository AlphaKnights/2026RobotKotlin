package frc.robot.commands.intake

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.Constants

class IntakeLeverManualCommand(
    private val direction: Constants.IntakeDirection,
) : Command(){


    init {
        addRequirements(IntakeSubsystem)
    }

    override fun initialize() {
        super.initialize()
    }
    override fun execute() {
        val speed =
            when (direction) {
                Constants.IntakeDirection.IN ->
                    Constants.IntakeConstants.LEVER_SPEED
                Constants.IntakeDirection.OUT ->
                    -Constants.IntakeConstants.LEVER_SPEED
            }
        IntakeSubsystem.moveLever(speed)
    }

    override fun isFinished(): Boolean {
        return IntakeSubsystem.limitSwitchPressed()
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopIntakeLever()

    }

}