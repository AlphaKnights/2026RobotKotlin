package frc.robot.subsystems

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.ModuleConstants

class SwerveModuleSim : SwerveModule {
    private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val driveSim =
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
                ModuleConstants.DRIVE_RATIO, // PLEASE CHANGE THIS IT IS SO WRONG
            ),
            TURN_GEARBOX,
        )

    private val driveController =
//        PIDController(
//            1.0,
//            1.0,
//            1.0,
//        )
        SmartDashboard.getData("DrivePID") as PIDController
    private val turnController =
        PIDController(
            DriveConstants.ROTATE_CONTROLLER_P,
            DriveConstants.ROTATE_CONTROLLER_I,
            DriveConstants.ROTATE_CONTROLLER_D,
        )

    init {

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun getPosition(): SwerveModulePosition =
        SwerveModulePosition(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveSim.angularPositionRotations,
            Rotation2d.fromRotations(
                turnSim.angularPositionRotations,
            ),
        )

    override fun getState(): SwerveModuleState =
        SwerveModuleState(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveSim.angularVelocityRPM * 60,
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
                driveSim.angularVelocityRPM * 60,
                desiredState.speedMetersPerSecond / ModuleConstants.WHEEL_CIRCUMFERENCE,
            )
        SmartDashboard.putNumber("driveAppliedVolts", driveAppliedVolts)
        val turnAppliedVolts: Double =
            turnController.calculate(
                turnSim.angularPositionRotations,
                desiredState.angle.rotations,
            )

        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0))
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0))

        turnSim.update(0.02)
        driveSim.update(0.02)
    }
}
