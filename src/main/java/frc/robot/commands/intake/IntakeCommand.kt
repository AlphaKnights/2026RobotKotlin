package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand() : Command() {
    init {
        addRequirements(IntakeSubsystem)
    }

    override fun execute() {
        IntakeSubsystem.runMotor()
    }



    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopMotor()
    }
}