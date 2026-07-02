package frc.robot.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkFlexConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants.ModuleConstants
import kotlin.math.PI

class SwerveModuleIOSparkMAX(
    drivingCANId: Int,
    turningCANId: Int,
    chassisAngularOffset: Rotation2d,
) : SwerveModule {
    private val m_drivingSpark = SparkFlex(drivingCANId, SparkLowLevel.MotorType.kBrushless)
    private val m_turningSpark = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)

    private val m_drivingEncoder: RelativeEncoder = m_drivingSpark.encoder
    private val m_turningEncoder: AbsoluteEncoder = m_turningSpark.absoluteEncoder

    private val m_drivingClosedLoopController: SparkClosedLoopController = m_drivingSpark.getClosedLoopController()
    private val m_turningClosedLoopController: SparkClosedLoopController = m_turningSpark.getClosedLoopController()

    private var m_chassisAngularOffset = 0.0
    private var m_desiredState = SwerveModuleState(0.0, Rotation2d())

    private val drivingConfig: SparkFlexConfig = SparkFlexConfig()

    private val turningConfig: SparkMaxConfig = SparkMaxConfig()

    // Use module constants to calculate conversion factors and feed forward gain.
    private val drivingFactor = ModuleConstants.WHEEL_CIRCUMFERENCE / ModuleConstants.DRIVE_RATIO
    private val turningFactor = 2 * PI
    private val drivingVelocityFeedForward = ModuleConstants.DRIVING_FF

    override fun getState(): SwerveModuleState {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return SwerveModuleState(
            m_drivingEncoder.velocity,
            Rotation2d(m_turningEncoder.position - m_chassisAngularOffset),
        )
    }

    override fun getPosition(): SwerveModulePosition {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return SwerveModulePosition(
            m_drivingEncoder.position,
            Rotation2d(m_turningEncoder.position - m_chassisAngularOffset),
        )
    }

    override fun setDesiredState(desiredState: SwerveModuleState) {
        // Apply chassis angular offset to the desired state.
        val correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d(m_turningEncoder.position))

        // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity)
        m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.radians, ControlType.kPosition)

        m_desiredState = desiredState
    }

    /** Zeroes all the SwerveModule encoders.  */
    fun resetEncoders() {
        m_drivingEncoder.position = 0.0
    }
}
