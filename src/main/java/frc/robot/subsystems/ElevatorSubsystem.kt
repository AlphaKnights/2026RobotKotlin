/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ElevatorConstants

object ElevatorSubsystem : SubsystemBase() {
    private val leftMotor =
        SparkMax(
            ElevatorConstants.LEFT_MOTOR_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )
    private val rightMotor =
        SparkMax(
            ElevatorConstants.RIGHT_MOTOR_CAN_ID,
            SparkLowLevel.MotorType.kBrushless,
        )

    private val leftPIDController = leftMotor.closedLoopController
    private val rightPIDController = rightMotor.closedLoopController

    init {
        val leftConfig =
            SparkMaxConfig().apply {
                inverted(true)
                idleMode(ElevatorConstants.IDLE_MODE)
                smartCurrentLimit(
                    ElevatorConstants.CURRENT_LIMIT,
                )

                softLimit.apply {
                    forwardSoftLimitEnabled(true)
                    reverseSoftLimitEnabled(true)

                    forwardSoftLimit(
                        ElevatorConstants.FORWARD_SOFT_LIMIT,
                    )
                    reverseSoftLimit(
                        ElevatorConstants.REVERSE_SOFT_LIMIT,
                    )
                }

                encoder.apply {
                    positionConversionFactor(1.0)
                    velocityConversionFactor(1.0)
                }

                closedLoop.apply {
                    feedbackSensor(
                        ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder,
                    )
                    pid(
                        ElevatorConstants.P,
                        ElevatorConstants.I,
                        ElevatorConstants.D,
                    )
                    outputRange(-1.0, 1.0)
                    positionWrappingEnabled(false)
                }
            }

        val rightConfig =
            SparkMaxConfig().apply {
                inverted(false)
                idleMode(ElevatorConstants.IDLE_MODE)
                smartCurrentLimit(
                    ElevatorConstants.CURRENT_LIMIT,
                )

                softLimit.apply {
                    forwardSoftLimitEnabled(true)
                    reverseSoftLimitEnabled(true)

                    forwardSoftLimit(
                        ElevatorConstants.FORWARD_SOFT_LIMIT,
                    )
                    reverseSoftLimit(
                        ElevatorConstants.REVERSE_SOFT_LIMIT,
                    )
                }

                encoder.apply {
                    positionConversionFactor(1.0)
                    velocityConversionFactor(1.0)
                }

                closedLoop.apply {
                    feedbackSensor(
                        ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder,
                    )
                    pid(
                        ElevatorConstants.P,
                        ElevatorConstants.I,
                        ElevatorConstants.D,
                    )
                    outputRange(-1.0, 1.0)
                    positionWrappingEnabled(false)
                }
            }

        leftMotor.configure(
            leftConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        rightMotor.configure(
            rightConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
    }

    /**
     * Sets the speed of the elevator motors.
     * @param speed The proportion speed to set the motors to, between -1.0 and 1.0.
     */
    fun move(speed: Double) {
        leftMotor.set(speed)
        rightMotor.set(speed)
    }

    /**
     * Sets the elevator motors to a specific position.
     * @param position The position to set the motors to, in rotations.
     */
    fun setPosition(position: Double) {
        leftPIDController.setReference(
            position,
            SparkBase.ControlType.kPosition,
        )
        rightPIDController.setReference(
            position,
            SparkBase.ControlType.kPosition,
        )
    }

    /**
     * Gets the current position of the elevator.
     * @return The current position of the elevator, in rotations.
     */
    fun getPosition(): Double = (leftMotor.encoder.position)

    fun stop() {
        leftMotor.stopMotor()
        rightMotor.stopMotor()
    }
}
