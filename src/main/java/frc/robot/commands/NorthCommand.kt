package frc.robot.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem

class NorthCommand(
    private val x: () -> Double,
    private val y: () -> Double,
) : Command() {

    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        // take current rotation in radians and make a new PID Controller
        val curpose = DriveSubsystem.getPose().rotation.radians
        val controller = PIDController(1.0, 0.0, 1.0)

        // set PID deadzones and angle wrapping
        controller.setTolerance(Rotation2d.fromDegrees(1.0).radians)
        controller.enableContinuousInput(0.0, 2 * Math.PI)

        // calculate rotational speed using PID controller, making sure max speed is respected
        val rotSpeed =
            clamp(
                controller.calculate(curpose, 0.0),
                0.0,
                1.0,
            ) * Constants.DriveConstants.MAX_ANGULAR_SPEED

        DriveSubsystem.drive(
            ChassisSpeeds(
                x() *
                        Constants.DriveConstants.MAX_METERS_PER_SECOND,
                y() *
                        Constants.DriveConstants.MAX_METERS_PER_SECOND,
                rotSpeed,
            ),
            fieldRelative = true,
        )
    }

    override fun isFinished(): Boolean = false
}
