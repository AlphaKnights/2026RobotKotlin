package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand() : Command() {
    init {
        addRequirements(IntakeSubsystem)
    }

    override fun execute() {
        println("RUNNING INTAKE MOTOR _______________________")
        IntakeSubsystem.runMotor()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopMotor()
    }
}