package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand(
    private val isReversed: Boolean,
) : Command(){
    init {
        addRequirements(IntakeSubsystem)
    }

    override fun initialize() {
        super.initialize()
    }

    override fun execute() {
        IntakeSubsystem.limitOutput()
        val intakeSpeed =
            if (isReversed) {
                -Constants.IntakeConstants.INTAKE_SPEED
            } else Constants.IntakeConstants.INTAKE_SPEED

        IntakeSubsystem.runIntake(intakeSpeed)

    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopIntake()

    }

    }