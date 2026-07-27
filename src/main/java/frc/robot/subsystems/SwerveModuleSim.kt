/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.Constants.ModuleConstants
import frc.robot.interfaces.SwerveModule
import frc.robot.subsystems.Logger.initTunablePID

class SwerveModuleSim : SwerveModule {
    private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)

    val driveSim =
        DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX,
                2.0,
                ModuleConstants.DRIVE_RATIO,
            ),
            DRIVE_GEARBOX,
        )
    private val turnSim =
        DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX,
                2.0,
                ModuleConstants.DRIVE_RATIO, // PLEASE CHANGE THIS TO ACTUAL VALUE
            ),
            TURN_GEARBOX,
        )

    val driveController = PIDController(1.0, 0.0, 0.0)

    val turnController = PIDController(1.0, 0.0, 0.0)

    init {
        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI)
        initTunablePID("DrivePID", driveController)
        initTunablePID("TurnPID", turnController)
    }

    override fun getPosition(): SwerveModulePosition =
        SwerveModulePosition(
            driveSim.angularPositionRotations * ModuleConstants.WHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(
                turnSim.angularPositionRotations,
            ),
        )

    override fun getState(): SwerveModuleState =
        SwerveModuleState(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveSim.angularVelocityRPM / 60,
            Rotation2d.fromRotations(
                turnSim.angularPositionRotations,
            ),
        )

    override fun setDesiredState(desiredState: SwerveModuleState) {
        desiredState.optimize(
            Rotation2d.fromRotations(
                turnSim.angularPositionRotations,
            ),
        )

        val driveAppliedVolts: Double =
            driveController.calculate(
                driveSim.angularVelocityRPM / 60,
                desiredState.speedMetersPerSecond / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE,
            )
        SmartDashboard.putNumber("driveAppliedVolts", driveAppliedVolts)
//        val turnAppliedVolts: Double =
//            turnController.calculate(
//                turnSim.angularPositionRotations,
//                desiredState.angle.rotations,
//            ) + (turnSim.angularAccelerationRadPerSecSq * turn_KV * Math.PI)

//        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0))
        turnSim.setAngle(desiredState.angle.radians)
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0))

        turnSim.update(0.02)
        driveSim.update(0.02)
    }
}
