package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


object IntakeSubsystem : SubsystemBase() {
    private val intakeMotor =
        SparkMax(
            Constants.IntakeConstants.INTAKE_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )

    private val leverMotor =
        SparkMax(
            Constants.IntakeConstants.LEVER_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )

    val pidController = intakeMotor.closedLoopController

    init {
        val intakeMotorConfig =
            SparkMaxConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kBrake)
            }

        val leverMotorConfig =
            SparkMaxConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kBrake)
                softLimit.apply {
                    forwardSoftLimitEnabled(true)
                    reverseSoftLimitEnabled(true)

                    forwardSoftLimit(Constants.IntakeConstants.LEVER_LIMIT_FORWARD)
                    reverseSoftLimit(Constants.IntakeConstants.LEVER_LIMIT_REVERSE)
                }

                encoder.apply {
                    positionConversionFactor(1.0) // rotations
                    velocityConversionFactor(1.0) // rotations
                }

                closedLoop.apply {
                    feedbackSensor(
                        ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder,
                    )
                    pid(
                        Constants.IntakeConstants.P,
                        Constants.IntakeConstants.I,
                        Constants.IntakeConstants.D,
                    )
                    outputRange(-1.0, 1.0)
                    positionWrappingEnabled(false)
                }
            }
        intakeMotor.configure(
            intakeMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
        leverMotor.configure(
            leverMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
    }

    fun runIntake(speed: Double) {
        intakeMotor.set(speed)
    }

    fun setPosition(position: Double) {
        pidController.setReference(
            position,
            SparkBase.ControlType.kPosition,
            )
    }

    fun moveLever(speed: Double) {
        pidController.setReference(
            speed,
            SparkBase.ControlType.kVelocity // maybe change this, units are rpm right now
        )
    }

    fun getPosition(): Double = leverMotor.encoder.getPosition()

    fun stopIntake() {
        intakeMotor.stopMotor()
    }



}







