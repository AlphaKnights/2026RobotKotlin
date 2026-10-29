package frc.robot.commands
import frc.robot.Constants
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DeliverySubsystem

class DeliveryCommand() : Command() {
    constructor(launchSpeed: Double) : this()

    init {

        addRequirements(DeliverySubsystem)
    }

    override fun execute() {
        DeliverySubsystem.runDelivery()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
    }
}