package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DeliverySubsystem

class DeliveryCommand : Command() {

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(DeliverySubsystem)

    }

    override fun initialize() {
        super.initialize()
    }

    override fun execute() {
        DeliverySubsystem.forward(Constants.LaunchConstants.LAUNCH_SPEED)

    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        DeliverySubsystem.stop()
    }
}
