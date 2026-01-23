/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ElevatorSubsystem

class ElevatorPosCommand(
    private val targetPosition: Double,
) : Command() {
    init {
        addRequirements(ElevatorSubsystem)
    }

    override fun execute() {
        ElevatorSubsystem.setPosition(targetPosition)
    }

    override fun isFinished(): Boolean = false
}
