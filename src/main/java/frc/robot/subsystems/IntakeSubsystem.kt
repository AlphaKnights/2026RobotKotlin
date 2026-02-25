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

    private val rightleverMotor =
        SparkMax(
            Constants.IntakeConstants.RIGHT_LEVER_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )

    private val leftLeverMotor =
        SparkMax(
            Constants.IntakeConstants.LEFT_LEVER_MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )




    val pidController = intakeMotor.closedLoopController

    init {
        val intakeMotorConfig=
            SparkMaxConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kBrake)
            }

        val globalConfig =
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

        val leftConfig = SparkMaxConfig().apply {
            follow(rightleverMotor)
            apply(globalConfig)
        }
        val rightConfig = SparkMaxConfig().apply {
            apply(globalConfig)
        }
        rightleverMotor.configure( //the right one is leading
            rightConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )

        leftLeverMotor.configure(
            leftConfig,
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







