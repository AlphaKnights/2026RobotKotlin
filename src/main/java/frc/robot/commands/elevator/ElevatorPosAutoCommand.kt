/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.ElevatorSubsystem
import kotlin.math.abs

class ElevatorPosAutoCommand(
    private val targetPosition: Double,
) : Command() {
    init {
        addRequirements(ElevatorSubsystem)
    }

    override fun execute() {
        ElevatorSubsystem.setPosition(targetPosition)
    }

    override fun isFinished(): Boolean =
        abs(ElevatorSubsystem.getPosition() - targetPosition) < Constants.ElevatorConstants.POS_DEADZONE
}
