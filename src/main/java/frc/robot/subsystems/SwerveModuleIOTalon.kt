/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants.ModuleConstants
import frc.robot.interfaces.SwerveModule

class SwerveModuleIOTalon(
    driveMotorId: Int,
    turnMotorId: Int,
    encoderId: Int,
    private val offset: Rotation2d,
) : SwerveModule {
    private val driveMotor = TalonFX(driveMotorId)
    private val turnMotor = TalonFX(turnMotorId)
    private val encoder = CANcoder(encoderId)
    private var desiredState =
        SwerveModuleState(
            0.0,
            Rotation2d.fromRotations(
                encoder.position.valueAsDouble,
            ) +
                offset,
        )

    override fun getPosition(): SwerveModulePosition =
        SwerveModulePosition(
            // driveMotor.rotor
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveMotor.position.valueAsDouble,
            Rotation2d.fromRotations(
                turnMotor.position.valueAsDouble,
            ) +
                offset,
        )

    override fun getState(): SwerveModuleState =
        SwerveModuleState(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveMotor.velocity.valueAsDouble,
            Rotation2d.fromRotations(
                turnMotor.position.valueAsDouble,
            ) +
                offset,
        )

    override fun setDesiredState(desiredState: SwerveModuleState) {
        val correctedState =
            SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle - offset,
            )
        correctedState.optimize(
            Rotation2d.fromRotations(
                encoder.position.valueAsDouble,
            ),
        )

        driveMotor.setControl(
            VelocityVoltage(
                correctedState.speedMetersPerSecond / ModuleConstants.WHEEL_CIRCUMFERENCE,
            ),
        )
        turnMotor.setControl(
            PositionVoltage(correctedState.angle.rotations),
        )

        this.desiredState = desiredState
    }
}
