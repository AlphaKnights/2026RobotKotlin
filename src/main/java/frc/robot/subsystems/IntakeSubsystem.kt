package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


object IntakeSubsystem : SubsystemBase() {
    private val intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID)

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

    val rightPidController = rightleverMotor.closedLoopController
    val leftPidController = leftLeverMotor.closedLoopController

    init {
        val intakeMotorConfig=
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT
                }

                MotorOutput.apply{
                    NeutralMode = NeutralModeValue.Brake
                }
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
                        FeedbackSensor.kPrimaryEncoder,
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
        intakeMotor.getConfigurator().apply(intakeMotorConfig)

        val leftConfig = SparkMaxConfig().apply {
            follow(rightleverMotor)
            apply(globalConfig)
        }
        val rightConfig = SparkMaxConfig().apply {
            apply(globalConfig)
        }
        rightleverMotor.configure( //the right one is leading
            rightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

        leftLeverMotor.configure(
            leftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters,
        )

    }

    fun runIntake(speed: Double) {
        intakeMotor.set(speed)
    }

    fun setPosition(position: Double) {
        rightPidController.setSetpoint(
            position,
            SparkBase.ControlType.kPosition,
            )
        leftPidController.setSetpoint(
            position,
            SparkBase.ControlType.kPosition,
        )
    }

    fun moveLever(speed: Double) {
        rightPidController.setSetpoint(
            speed,
            SparkBase.ControlType.kVelocity // maybe change this, units are rpm right now
        )
        leftPidController.setSetpoint(
            speed,
            SparkBase.ControlType.kVelocity // maybe change this, units are rpm right now
        )
    }

    fun getPosition(): Double = rightleverMotor.encoder.getPosition()

    fun stopIntake() {
        intakeMotor.stopMotor()
    }



}







