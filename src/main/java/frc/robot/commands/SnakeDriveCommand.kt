package frc.robot.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem


class SnakeDriveCommand(
    private val x: () ->  Double,
    private val y: () -> Double,
    private val angle: () -> Double
) : Command() {

    init {
        addRequirements(DriveSubsystem)
    }

    val rotateController = PIDController(1.0, 0.0, 0.01)
    val driveController = PIDController(0.5, 0.0, 0.0)

    // Do angle optimization (south) and scalable tuning based on max speed
    override fun execute() {
        super.execute()
        val curpose = DriveSubsystem.getPose()


        // set PID deadzones and angle wrapping
        rotateController.setTolerance(Rotation2d.fromRadians(1.0).radians)
        rotateController.enableContinuousInput(-Math.PI, Math.PI)

        driveController.setTolerance(0.05) // meters


        // calculate rotational speed using PID controller, making sure max speed is respected
        val rotSpeed =
            clamp(
                rotateController.calculate(curpose.rotation.radians, angle()),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_ANGULAR_SPEED

        val driveSpeedX =
            clamp(
                driveController.calculate(curpose.translation.x, x()),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_METERS_PER_SECOND

        val driveSpeedY =
            clamp(
                driveController.calculate(curpose.translation.y, y()),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_METERS_PER_SECOND

        DriveSubsystem.drive(
            ChassisSpeeds(
                driveSpeedX,
                driveSpeedY,
                rotSpeed,
            ),
            fieldRelative = true,
        )
    }

}