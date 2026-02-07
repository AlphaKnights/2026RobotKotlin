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
            )
        )

        configureBindings()
    }


    private fun configureBindings() {
        // Drive control
        //x is forward
        DriveSubsystem.defaultCommand = DriveCommand(
            x = { if (kotlin.math.abs(xBoxController.x()) > 0.2) {xBoxController.x()} else 0.0},
            y = { if (kotlin.math.abs(xBoxController.y()) > 0.2) { xBoxController.y()} else 0.0},
            rot = { if (kotlin.math.abs(xBoxController.rot()) > 0.2) { xBoxController.rot()} else 0.0}
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


    }

    fun getAutonomousCommand(): Command {

        //return commands2.SequentialCommandGroup(commands2.InstantCommand(lambda: self.robotDrive.drive(ChassisSpeeds(-8, 0, 0), False, False), self.robotDrive),
        //        #                                         commands2.WaitCommand(AutoConstants.kTimedTime),
        //        #                                         commands2.InstantCommand(lambda: self.robotDrive.drive(ChassisSpeeds(0, 0, 0), False, False), self.robotDrive)
        //        #                                         )


        return PathPlannerAuto("Auto Activity")
    }
}