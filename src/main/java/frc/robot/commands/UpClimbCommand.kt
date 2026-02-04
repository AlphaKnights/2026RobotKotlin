package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
class UpClimbCommand(private val UpAmount: Double): Command() {
    init {
        addRequirements(ClimbSubsystem)
    }

    override fun initialize() {
        super.initialize()
    }

    override fun execute() {
        ClimbSubsystem.setPosition(UpAmount)
    }

    override fun isFinished(): Boolean {
        return false
    }
    override fun end(interrupted: Boolean) {
        super.end(interrupted)
    }
}