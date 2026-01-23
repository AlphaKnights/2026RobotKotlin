package frc.robot
import kotlin.math.*

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import frc.robot.commands.DriveCommand
import frc.robot.commands.ResetHeadingCommand
import frc.robot.commands.autoalign.AutoAlignAutoCommand
import frc.robot.commands.autoalign.AutoAlignManualCommand
import frc.robot.commands.coralmanipulator.IntakeCommand
import frc.robot.commands.coralmanipulator.LaunchCommand
import frc.robot.commands.elevator.ElevatorManualCommand
import frc.robot.commands.elevator.ElevatorPosAutoCommand
import frc.robot.commands.elevator.ElevatorPosCommand
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer
{
    private val joystickController = JoystickController()

    private val buttonBoard = CommandJoystick(Constants.OperatorConstants.BUTTON_BOARD_PORT)

    init
    {
        LimelightSubsystem.startPolling()

        NamedCommands.registerCommands(
            mapOf(
                "Left" to AutoAlignAutoCommand(Constants.AlignDirection.LEFT),
                "Right" to AutoAlignAutoCommand(Constants.AlignDirection.RIGHT),

                "Lvl 1" to ElevatorPosAutoCommand(Constants.ElevatorConstants.LVL_1_HEIGHT),
                "Lvl 2" to ElevatorPosAutoCommand(Constants.ElevatorConstants.LVL_2_HEIGHT),
                "Lvl 3" to ElevatorPosAutoCommand(Constants.ElevatorConstants.LVL_3_HEIGHT),
                "Lvl 4" to ElevatorPosAutoCommand(Constants.ElevatorConstants.LVL_4_HEIGHT),

                "Intake" to IntakeCommand(),
                "Delivery" to LaunchCommand(),
            )
        )

        configureBindings()
    }

    private fun configureBindings() {
        // Drive control

        //x is forward
        DriveSubsystem.defaultCommand = DriveCommand(
            x = { if (kotlin.math.abs(joystickController.x()) > 0.2) {joystickController.x()} else 0.0},
            y = { if (kotlin.math.abs(joystickController.y()) > 0.2) {joystickController.y()} else 0.0},
            rot = { if (kotlin.math.abs(joystickController.rot()) > 0.2) {joystickController.rot()} else 0.0}
        )

        // Reset heading
        joystickController.heading()
            .whileTrue(
                ResetHeadingCommand()
            )
        // Auto Align
        joystickController.alignL().whileTrue(
            AutoAlignManualCommand(
                Constants.AlignDirection.LEFT,
            )
        )
        joystickController.alignR().whileTrue(AutoAlignManualCommand(
            Constants.AlignDirection.RIGHT,
        ))

        // Manual elevator control
        buttonBoard.button(Constants.OperatorConstants.ELEVATOR_UP_BUTTON)
            .whileTrue(
                ElevatorManualCommand(
                    Constants.ElevatorDirection.UP
                )
            )

        buttonBoard.button(Constants.OperatorConstants.ELEVATOR_DOWN_BUTTON)
            .whileTrue(
                ElevatorManualCommand(
                    Constants.ElevatorDirection.DOWN
                )
            )

        // Elevator positioning
        buttonBoard.button(Constants.OperatorConstants.ELEVATOR_LVL_1_BUTTON)
            .onTrue(
                ElevatorPosCommand(
                    Constants.ElevatorConstants.LVL_1_HEIGHT
                )
            )

        buttonBoard.button(Constants.OperatorConstants.ELEVATOR_LVL_2_BUTTON)
            .onTrue(
                ElevatorPosCommand(
                    Constants.ElevatorConstants.LVL_2_HEIGHT
                )
            )

        buttonBoard.button(Constants.OperatorConstants.ELEVATOR_LVL_3_BUTTON)
            .onTrue(
                ElevatorPosCommand(
                    Constants.ElevatorConstants.LVL_3_HEIGHT
                )
            )

        buttonBoard.button(Constants.OperatorConstants.ELEVATOR_LVL_4_BUTTON)
            .onTrue(
                ElevatorPosCommand(
                    Constants.ElevatorConstants.LVL_4_HEIGHT
                )
            )

        // Coral Manipulator
        buttonBoard.button(Constants.OperatorConstants.DELIVERY_BUTTON)
            .onTrue(
                LaunchCommand()
            )
        buttonBoard.button(Constants.OperatorConstants.INTAKE_BUTTON)
            .onTrue(
                IntakeCommand()
            )
    }

    fun getAutonomousCommand(): Command {

        //return commands2.SequentialCommandGroup(commands2.InstantCommand(lambda: self.robotDrive.drive(ChassisSpeeds(-8, 0, 0), False, False), self.robotDrive),
        //        #                                         commands2.WaitCommand(AutoConstants.kTimedTime),
        //        #                                         commands2.InstantCommand(lambda: self.robotDrive.drive(ChassisSpeeds(0, 0, 0), False, False), self.robotDrive)
        //        #                                         )


        return PathPlannerAuto("Auto Activity")
    }
}