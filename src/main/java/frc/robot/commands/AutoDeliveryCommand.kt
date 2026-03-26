package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DeliverySubsystem

class AutoDeliveryCommand(deliverySpeed: Double) : Command() {

    val timer: Timer = Timer()
    val speed = deliverySpeed

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(DeliverySubsystem)

    }

    override fun initialize() {
        super.initialize()
        timer.start()
    }

    override fun execute() {
        DeliverySubsystem.forward(speed)

    }

    override fun isFinished(): Boolean {
        if (timer.get() > 3){
            return true
        }
        return false
    }

    override fun end(interrupted: Boolean) {
        DeliverySubsystem.stop()
        timer.reset()
        timer.stop()
    }
}
