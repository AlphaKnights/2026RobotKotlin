package frc.robot
import kotlin.math.*

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import frc.robot.commands.DriveCommand
import frc.robot.commands.ResetHeadingCommand
import frc.robot.commands.autoalign.AutoAlignAutoCommand
import frc.robot.commands.autoalign.AutoAlignManualCommand
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.XBoxController
import frc.robot.commands.DeliveryCommand
import frc.robot.commands.intake.*

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
    //private val joystickController = JoystickController()
    private val xBoxController = XBoxController()


    private val buttonBoard = CommandJoystick(Constants.OperatorConstants.BUTTON_BOARD_PORT)

    init
    {
        LimelightSubsystem.startPolling()

        NamedCommands.registerCommands(
            mapOf(
                "Left" to AutoAlignAutoCommand(Constants.AlignDirection.LEFT),
                "Right" to AutoAlignAutoCommand(Constants.AlignDirection.RIGHT),
                "Delivery" to DeliveryCommand(),
                "Intake_Lever_In" to IntakeLeverCommand(Constants.IntakeConstants.LEVER_IN_POSITION),
                "Intake_Lever_Out" to IntakeLeverCommand(Constants.IntakeConstants.LEVER_OUT_POSITION),
                "Intake" to IntakeCommand(false)

            )
        )

        configureBindings()
    }

    private fun configureBindings() {
        // Drive control

        //x is forward
        DriveSubsystem.defaultCommand = DriveCommand(
            x = {xBoxController.x()},
            y = {xBoxController.y()},
            rot = {xBoxController.rot()},
            autoAngle = { xBoxController.autoAim().getAsBoolean()}
        )

        // Reset heading
        xBoxController.heading()
            .whileTrue(
                ResetHeadingCommand()
            )
        // Auto Align
        xBoxController.alignL().whileTrue(
            AutoAlignManualCommand(
                Constants.AlignDirection.LEFT,
            )
        )
        xBoxController.alignR().whileTrue(AutoAlignManualCommand(
            Constants.AlignDirection.RIGHT,
        ))

        buttonBoard.button(Constants.OperatorConstants.DELIVERY_BUTTON)
            .whileTrue(
                DeliveryCommand()
            )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_IN_AUTO_BUTTON
            ).onTrue(
                IntakeLeverCommand(
                    Constants.IntakeConstants.LEVER_IN_POSITION
                )
            )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_OUT_AUTO_BUTTON
            ).onTrue(
                IntakeLeverCommand(
                    Constants.IntakeConstants.LEVER_OUT_POSITION
                )
            )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_IN_MANUAL_BUTTON
            ).whileTrue(
                IntakeLeverManualCommand(
                    Constants.IntakeDirection.IN
                )
            )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_OUT_MANUAL_BUTTON
            ).whileTrue(
                IntakeLeverManualCommand(
                    Constants.IntakeDirection.OUT
                )
            )

        buttonBoard.button(Constants.OperatorConstants.INTAKE_BUTTON).whileTrue(IntakeCommand(false))




    }

    fun getAutonomousCommand(): Command {

        //return commands2.SequentialCommandGroup(commands2.InstantCommand(lambda: self.robotDrive.drive(ChassisSpeeds(-8, 0, 0), False, False), self.robotDrive),
        //        #                                         commands2.WaitCommand(AutoConstants.kTimedTime),
        //        #                                         commands2.InstantCommand(lambda: self.robotDrive.drive(ChassisSpeeds(0, 0, 0), False, False), self.robotDrive)
        //        #                                         )


        return PathPlannerAuto("Auto Activity")
    }
}