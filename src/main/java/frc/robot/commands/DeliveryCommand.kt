package frc.robot.commands

import frc.robot.Constants
import frc.robot.subsystems.DeliverySubsystem
import org.wpilib.wpilibj.Timer
import org.wpilib.wpilibj2.command.Command

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
