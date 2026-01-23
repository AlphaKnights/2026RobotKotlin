/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.elevator

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.ElevatorSubsystem

class ElevatorManualCommand(
    private val direction: Constants.ElevatorDirection,
) : Command() {
    init {
        addRequirements(ElevatorSubsystem)
    }

    override fun execute() {
        val speed: Double =
            when (direction) {
                Constants.ElevatorDirection.UP ->
                    Constants.ElevatorConstants.MAX_ELEVATOR_SPEED
                Constants.ElevatorDirection.DOWN ->
                    -Constants.ElevatorConstants.MAX_ELEVATOR_SPEED
            }

        ElevatorSubsystem.move(speed)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        ElevatorSubsystem.stop()
    }
}
