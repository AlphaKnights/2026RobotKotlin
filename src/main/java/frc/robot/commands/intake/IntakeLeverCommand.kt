/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.IntakeSubsystem

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
