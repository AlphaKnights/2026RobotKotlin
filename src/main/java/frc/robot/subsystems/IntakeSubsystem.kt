/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import java.util.function.DoubleConsumer

object IntakeSubsystem : SubsystemBase() {
    private val CAN = Constants.ModuleConstants.CANBUS
    private val intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, CAN)

    val rightleverMotor =
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

            MotorOutput.apply {
                InvertedValue.CounterClockwise_Positive
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
        rightleverMotor.configurator.apply(globalConfig)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSafeState {
            this.rightleverMotor.disable()
            this.intakeMotor.disable()
        }
        builder.addDoubleProperty("position", ::getPosition, ::setPosition)
        builder.addDoubleProperty("lever voltage", { rightleverMotor.motorVoltage.valueAsDouble }, null)
        builder.addDoubleProperty("setpoint", { rightleverMotor.closedLoopReference.valueAsDouble }, null)
        builder.addDoubleProperty("control error", { rightleverMotor.closedLoopError.valueAsDouble }, null)

        builder.addDoubleProperty(
            "P",
            {
                globalConfig.Slot0.kP
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKP(value))
            },
        )

        builder.addDoubleProperty(
            "I",
            {
                globalConfig.Slot0.kI
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKI(value))
            },
        )

        builder.addDoubleProperty(
            "D",
            {
                globalConfig.Slot0.kD
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKD(value))
            },
        )

        builder.addDoubleProperty(
            "kG",
            {
                globalConfig.Slot0.kG
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKG(value))
            },
        )
    }

    fun runIntake(speed: Double) {
        intakeMotor.set(-speed)
    }

    fun setPosition(position: Double) {
        val mRequest = PositionDutyCycle(0.0).withSlot(0)

        rightleverMotor.setControl(mRequest.withPosition(-position))
        leftLeverMotor.setControl(Follower(rightleverMotor.deviceID, MotorAlignmentValue.Opposed))
    }

    fun isInPosition(deadzone: Double): Boolean = (rightleverMotor.closedLoopError.valueAsDouble < deadzone)

    fun moveLever(speed: Double) {
        rightleverMotor.set(speed)
        leftLeverMotor.set(-speed)
    }

    fun getPosition(): Double = rightleverMotor.position.valueAsDouble

    fun stopIntake() {
        intakeMotor.stopMotor()
    }

    fun stopIntakeLever() {
        rightleverMotor.stopMotor()
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
