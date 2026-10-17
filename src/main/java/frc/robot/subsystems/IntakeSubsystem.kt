/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.*
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import kotlin.math.absoluteValue

object IntakeSubsystem : SubsystemBase() {
    private val CAN = Constants.ModuleConstants.CANBUS
    private val intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, CAN)

    val rightLeverMotor =
        TalonFX(
            Constants.IntakeConstants.RIGHT_LEVER_MOTOR_ID,
            CAN,
        )

    val leftLeverMotor =
        TalonFX(
            Constants.IntakeConstants.LEFT_LEVER_MOTOR_ID,
            CAN,
        )

    // val limitUp: DigitalInput = DigitalInput(2)
    // val limitDown: DigitalInput = DigitalInput(3)

    val globalConfig =
        TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = Constants.IntakeConstants.INTAKE_STATOR_LIMIT
            }

            SoftwareLimitSwitch.apply {
                ForwardSoftLimitEnable
                ReverseSoftLimitEnable

                ForwardSoftLimitThreshold = Constants.IntakeConstants.LEVER_LIMIT_FORWARD
                ReverseSoftLimitThreshold = Constants.IntakeConstants.LEVER_LIMIT_REVERSE
            }

            Slot0.apply {
                kI = Constants.IntakeConstants.I
                kP = Constants.IntakeConstants.P
                kD = Constants.IntakeConstants.D
            }
            Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput
            ClosedLoopGeneral.GainSchedErrorThreshold = Constants.IntakeConstants.GAIN_SCHEDULE_ERROR_THRESHOLD
            Slot0.GravityType = GravityTypeValue.Arm_Cosine

            Feedback.SensorToMechanismRatio = 48.0


            MotorOutput.apply {
                InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }

            ResetMode.kResetSafeParameters
            PersistMode.kPersistParameters

            /*encoder.apply {
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
             */
        }

    init {
        val intakeMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = Constants.IntakeConstants.INTAKE_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }
            }

        intakeMotor.configurator.apply(intakeMotorConfig)
        leftLeverMotor.configurator.apply(globalConfig)
        rightLeverMotor.configurator.apply(globalConfig)


        // offset the start position so that 0.0 rotations is horizontal
        leftLeverMotor.setPosition(Constants.IntakeConstants.LEVER_IN_POSITION)
        rightLeverMotor.setPosition(Constants.IntakeConstants.LEVER_IN_POSITION)
        //rightLeverMotor.setControl(Constants.IntakeConstants.LEVER_IN_POSITION)

    }

    override fun initSendable(builder: SendableBuilder) {
        builder.apply {
//            setSafeState {
//                rightLeverMotor.disable()
//                intakeMotor.disable()
//            }
            addDoubleProperty("position", ::getPosition, null)
            addDoubleProperty("lever motor voltage", { rightLeverMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("lever stator current", { rightLeverMotor.statorCurrent.valueAsDouble.takeIf { it.absoluteValue >= Constants.TelemetryConstants.MOTOR_CURRENT_MIN_RANGE } ?: 0.0 }, null)
            addDoubleProperty("intake motor voltage", { intakeMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("intake stator current", { intakeMotor.statorCurrent.valueAsDouble.takeIf { it.absoluteValue >= Constants.TelemetryConstants.MOTOR_CURRENT_MIN_RANGE } ?: 0.0  }, null)
            addDoubleProperty("setpoint", { rightLeverMotor.closedLoopReference.valueAsDouble }, ::setPosition)
            addDoubleProperty("control error", { rightLeverMotor.closedLoopError.valueAsDouble }, null)

            addDoubleProperty(
                "P",
                {
                    globalConfig.Slot0.kP
                },
                { value: Double ->
                    rightLeverMotor.configurator.apply(globalConfig.Slot0.withKP(value))
                },
            )

            addDoubleProperty(
                "I",
                {
                    globalConfig.Slot0.kI
                },
                { value: Double ->
                    rightLeverMotor.configurator.apply(globalConfig.Slot0.withKI(value))
                },
            )

            addDoubleProperty(
                "D",
                {
                    globalConfig.Slot0.kD
                },
                { value: Double ->
                    rightLeverMotor.configurator.apply(globalConfig.Slot0.withKD(value))
                },
            )

            addDoubleProperty(
                "kG",
                {
                    globalConfig.Slot0.kG
                },
                { value: Double ->
                    rightLeverMotor.configurator.apply(globalConfig.Slot0.withKG(value))
                },
            )
        }
    }

    fun runIntake(speed: Double) {
        println("Intake Subsystem: RunIntake at $speed")
        intakeMotor.set(-speed)
    }

    fun setPosition(position: Double) {
        val mRequest = PositionDutyCycle(0.0).withSlot(0)

        rightLeverMotor.setControl(mRequest.withPosition(position))
        leftLeverMotor.setControl(Follower(rightLeverMotor.deviceID, MotorAlignmentValue.Opposed))
    }

    fun isInPosition(deadzone: Double): Boolean = (rightLeverMotor.closedLoopError.valueAsDouble < deadzone)

    fun moveLever(speed: Double) {
        rightLeverMotor.set(speed)
        leftLeverMotor.set(-speed)
    }

    fun getPosition(): Double = rightLeverMotor.position.valueAsDouble

    fun stopIntake() {
        println("Intake Subsystem: StopIntake")
        intakeMotor.stopMotor()
    }

    fun stopIntakeLever() {
        rightLeverMotor.stopMotor()
        leftLeverMotor.stopMotor()
    }

    fun limitSwitchPressed(): Boolean {
        // return (limitUp.get() || limitDown.get())
        return false
    }

    fun limitOutput() {
        // print("Limit Up Pressed: "+limitUp.get())
        // print("Limit Down Pressed: "+limitDown.get())
    }
}
