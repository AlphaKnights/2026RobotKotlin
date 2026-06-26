/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DeliverySubsystem

class DeliveryCommand(
    deliverySpeed: Double,
) : Command() {
    val speed = deliverySpeed

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(DeliverySubsystem)
    }

    override fun execute() {
        DeliverySubsystem.forward(speed)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        DeliverySubsystem.stop()
    }
}
