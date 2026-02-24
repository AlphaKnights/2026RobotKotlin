package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.ClimbSubsystem

class ManualClimbCommand(
    private val direction: Constants.ManualClimbDirection,
    ) : Command(){
    init { //initializes the subsystem
        addRequirements(ClimbSubsystem)
    }

    override fun execute() { //runs the command -> usually running subsystems methods
        val speed: Double =
            when (direction) {
                Constants.ManualClimbDirection.UP ->
                    Constants.ManualClimb.MAX_CLIMB_SPEED *
                            Constants.ManualClimb.MANUAL_SPEED_FACTOR
                Constants.ManualClimbDirection.DOWN ->
                    -Constants.ManualClimb.MAX_CLIMB_SPEED *
                            Constants.ManualClimb.MANUAL_SPEED_FACTOR
            }

        ClimbSubsystem.move(speed)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) { //run when the command is ended
        ClimbSubsystem.stop()

    }
}