package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ClimbConstants



object ClimbSystem {
    private val LeftArm = PWMSparkMax(ClimbConstants.LeftArmID)
    private val RightArm = PWMSparkMax(ClimbConstants.RightArmID)
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
                        ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder,
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
                        ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder,
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
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        RightArm.configure(
            rightConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
    }
}