package frc.robot.subsystems

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.ClimbConstants



object ClimbSubsystem : SubsystemBase() {
    private val LeftArm =
        SparkMax(
            Constants.ClimbConstants.LeftArmID,
            SparkLowLevel.MotorType.kBrushless
        )
    private val RightArm =
        SparkMax(
            Constants.ClimbConstants.RightArmID,
            SparkLowLevel.MotorType.kBrushless
        )

    private val LeftPIDController = LeftArm.closedLoopController
    private val RightPIDController = RightArm.closedLoopController


    init {
        val leftConfig =
            SparkMaxConfig().apply {
                inverted(true)
                idleMode(ClimbConstants.IDLE_MODE)
                smartCurrentLimit(
                    ClimbConstants.CURRENT_LIMIT,
                )

                softLimit.apply {
                    forwardSoftLimitEnabled(true)
                    reverseSoftLimitEnabled(true)

                    forwardSoftLimit(
                        ClimbConstants.FORWARD_SOFT_LIMIT,
                    )
                    reverseSoftLimit(
                        ClimbConstants.REVERSE_SOFT_LIMIT,
                    )
                }

                encoder.apply {
                    positionConversionFactor(1.0)
                    velocityConversionFactor(1.0)
                }

                closedLoop.apply {
                    feedbackSensor(
                        FeedbackSensor.kPrimaryEncoder, //ClosedLoopConfig.Feedback.Sensor
                    )
                    pid(
                        ClimbConstants.P,
                        ClimbConstants.I,
                        ClimbConstants.D,
                    )
                    outputRange(-1.0, 1.0)
                    positionWrappingEnabled(false)
                }
            }

        val rightConfig =
            SparkMaxConfig().apply {
                inverted(false)
                idleMode(ClimbConstants.IDLE_MODE)
                smartCurrentLimit(
                    ClimbConstants.CURRENT_LIMIT,
                )

                softLimit.apply {
                    forwardSoftLimitEnabled(true)
                    reverseSoftLimitEnabled(true)

                    forwardSoftLimit(
                        ClimbConstants.FORWARD_SOFT_LIMIT,
                    )
                    reverseSoftLimit(
                        ClimbConstants.REVERSE_SOFT_LIMIT,
                    )
                }

                encoder.apply {
                    positionConversionFactor(1.0)
                    velocityConversionFactor(1.0)
                }

                closedLoop.apply {
                    feedbackSensor(
                        FeedbackSensor.kPrimaryEncoder,
                    )
                    pid(
                        ClimbConstants.P,
                        ClimbConstants.I,
                        ClimbConstants.D,
                    )
                    outputRange(-1.0, 1.0)
                    positionWrappingEnabled(false)
                }
            }

        LeftArm.configure(
            leftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        RightArm.configure(
            rightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )




    }
    /**
     * Sets the speed of the elevator motors.
     * @param speed The proportion speed to set the motors to, between -1.0 and 1.0.
     */
    fun move(speed: Double) {
        LeftArm.set(speed)
        RightArm.set(speed)
    }

    /**
     * Sets the elevator motors to a specific position.
     * @param position The position to set the motors to, in rotations.
     */
    fun setPosition(position: Double) {
        LeftPIDController.setSetpoint(
            position,
            SparkBase.ControlType.kPosition,
        )
        RightPIDController.setSetpoint(
            position,
            SparkBase.ControlType.kPosition,
        )
    }

    /**
     * Gets the current position of the elevator.
     * @return The current position of the elevator, in rotations.
     */
    fun getPosition(): Double = (LeftArm.encoder.position)

    fun stop() {
        LeftArm.stopMotor()
        RightArm.stopMotor()
    }



}