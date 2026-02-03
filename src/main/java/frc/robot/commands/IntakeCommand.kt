package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem

class IntakeCommand : Command(){

    init {
        addRequirements(IntakeSubsystem);

    }

    override fun initialize() {
        super.initialize()
    }


    override fun execute() {
        IntakeSubsystem.forward(Constants.LaunchConstants.LAUNCH_SPEED)



    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stop()
        super.end(interrupted)

    }

    }