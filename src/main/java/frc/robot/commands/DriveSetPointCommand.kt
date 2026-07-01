/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.DriveConstants
import frc.robot.subsystems.DriveSubsystem

class DriveSetPointCommand(
    private val x: () -> Double,
    private val y: () -> Double,
    private val angle: () -> Double,
) : Command() {
    val rotateController =
        PIDController(
            DriveConstants.ROTATE_CONTROLLER_P,
            DriveConstants.ROTATE_CONTROLLER_I,
            DriveConstants.ROTATE_CONTROLLER_D,
        )
    val driveController =
        PIDController(
            DriveConstants.TRANSLATION_CONTROLLER_P,
            DriveConstants.TRANSLATION_CONTROLLER_I,
            DriveConstants.TRANSLATION_CONTROLLER_D,
        )

    init {
        addRequirements(DriveSubsystem)

        // configure the PID Controllers
        rotateController.setTolerance(Rotation2d.fromRadians(1.0).radians)
        rotateController.enableContinuousInput(-Math.PI, Math.PI)

        driveController.setTolerance(DriveConstants.DRIVE_SETPOINT_TOLERANCE)
    }

    // Do angle optimization (south) and scalable tuning based on max speed
    override fun execute() {
        super.execute()

        val curpose = DriveSubsystem.getPose()

//        val dir = when {
//            (curpose > Math.PI/2)  -> Math.PI
//            (curpose < -Math.PI/2) -> -Math.PI
//            else -> 0.0
//        }

        val rotSpeed =
            clamp(
                rotateController.calculate(curpose.rotation.radians, angle()),
                -1.0,
                1.0,
            ) * DriveConstants.MAX_ANGULAR_SPEED

        val driveSpeedX =
            clamp(
                driveController.calculate(curpose.translation.x, x()),
                -1.0,
                1.0,
            ) * DriveConstants.MAX_METERS_PER_SECOND

        val driveSpeedY =
            clamp(
                driveController.calculate(curpose.translation.y, y()),
                -1.0,
                1.0,
            ) * DriveConstants.MAX_METERS_PER_SECOND

        DriveSubsystem.drive(
            ChassisSpeeds(
                driveSpeedX,
                driveSpeedY,
                rotSpeed,
            ),
            fieldRelative = true,
        )
    }

    override fun isFinished(): Boolean =
        driveController.atSetpoint() &&
            rotateController.atSetpoint()
}
