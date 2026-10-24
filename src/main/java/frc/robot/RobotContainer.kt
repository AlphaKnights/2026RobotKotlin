/*
 * (C) 2025 Galvaknights
 */
package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import frc.robot.commands.*
import frc.robot.commands.DriveToArcCommand
import frc.robot.commands.autoalign.AutoAlignAutoCommand
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem
import java.io.File

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
object RobotContainer {
    // private val joystickController = JoystickController()
    private val xBoxController = XBoxController()

    private val buttonBoard = CommandJoystick(Constants.OperatorConstants.BUTTON_BOARD_PORT)

    private val autoChooser = SendableChooser<PathPlannerAuto>()

    init {
        LimelightSubsystem.startPolling()
        NamedCommands.registerCommands(
            mapOf(
                "Left" to AutoAlignAutoCommand(Constants.AlignDirection.LEFT),
                "Right" to AutoAlignAutoCommand(Constants.AlignDirection.RIGHT),
            ),
        )
        configureAuto()
        configureBindings()
    }

    private fun configureBindings() {
        // Drive control

        // x is forward
        DriveSubsystem.defaultCommand =
            DriveCommand(
                x = { xBoxController.x() },
                y = { xBoxController.y() },
                rot = { xBoxController.rot() },
                autoAngle = {
                    // xBoxController.autoAim().asBoolean
                    false
                },
            )

        // Reset heading
        xBoxController.heading().whileTrue(
            ResetHeadingCommand(),
        )
        // Auto Align
//        xBoxController.alignL().whileTrue(
//            AutoAlignManualCommand(
//                Constants.AlignDirection.LEFT,
//            ),
//        )
        //      xBoxController.alignR().whileTrue(
        //      AutoAlignManualCommand(
        //      Constants.AlignDirection.RIGHT,
        // ),
        // )

        xBoxController.resetOdometry().whileTrue(
            ResetOdometry(),
        )

        xBoxController
            .driveToArc()
            .onTrue(DriveToArcCommand())

//        xBoxController
//            .slideLeft()
//            .whileTrue(
//                DriveCommand(
//                    { 0.0 },
//                    {
//                        ArcSlidingCalc.getYChange(
//                            Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                                Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE,
//                        )
//                    },
//                    {
//                        Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                            Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE
//                    },
//                    { false },
//                    false,
//                ),
//            )
//
//        xBoxController
//            .slideRight()
//            .whileTrue(
//                DriveCommand(
//                    { 0.0 },
//                    {
//                        ArcSlidingCalc.getYChange(
//                            Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                                Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE,
//                        )
//                    },
//                    {
//                        -Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                            Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE
//                    },
//                    { false },
//                    false,
//                ),
//            )

        xBoxController.north().whileTrue(
            NorthCommand(
                x = { xBoxController.x() },
                y = { xBoxController.y() },
            ),
        )

        xBoxController.xLock().whileTrue(
            LockXCommand(),
        )


    }

    private fun configureAuto() {
        autoChooser.setDefaultOption("Default", PathPlannerAuto("Auto Deliver Only"))
        val autoList =
            buildList {
                File("src/main/deploy/pathplanner/autos/").listFiles()?.forEach { auto ->
                    add(PathPlannerAuto(auto.name.toString().removeSuffix(".auto")))
                }
            }
        for (auto in autoList) {
            autoChooser.addOption(auto.name, auto)
        }
        SmartDashboard.putData("Auto Chooser", autoChooser)
    }

    fun getAutonomousCommand(): Command = autoChooser.selected
}
