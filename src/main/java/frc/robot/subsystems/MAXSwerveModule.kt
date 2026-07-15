/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.PersistMode
import com.revrobotics.RelativeEncoder
import com.revrobotics.ResetMode
import com.revrobotics.spark.*
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkFlexConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.Constants.ModuleConstants
import kotlin.math.PI

/**
 * Constructs a MAXSwerveModule and configures the driving and turning motor,
 * encoder, and PID controller. This configuration is specific to the REV
 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
 * Encoder.
 */
class MAXSwerveModule(
    drivingCANId: Int,
    turningCANId: Int,
    chassisAngularOffset: Rotation2d,
) {
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

    init {
        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50) // KBRAKE IDLE
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0) // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
            .feedForward
            .kV(drivingVelocityFeedForward)

        turningConfig
            .idleMode(IdleMode.kBrake) // kBrake
            .smartCurrentLimit(20)
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0) // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, turningFactor)

        m_drivingSpark.configure(
            drivingConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )
        m_turningSpark.configure(
            turningConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        m_chassisAngularOffset = chassisAngularOffset.radians
        m_desiredState.angle = Rotation2d(m_turningEncoder.position)
        m_drivingEncoder.position = 0.0
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    fun getState(): SwerveModuleState {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return SwerveModuleState(
            m_drivingEncoder.velocity,
            Rotation2d(m_turningEncoder.position - m_chassisAngularOffset),
        )
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    fun getPosition(): SwerveModulePosition {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return SwerveModulePosition(
            m_drivingEncoder.position,
            Rotation2d(m_turningEncoder.position - m_chassisAngularOffset),
        )
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState) {
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
