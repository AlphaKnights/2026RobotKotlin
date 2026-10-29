/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.LaunchConstants
import frc.robot.Constants.SimConstants
import kotlin.math.absoluteValue

object DeliverySubsystem : SubsystemBase() {
    /*
    // Ultrasonic if we need one
    private val rangeFinder =
        Ultrasonic(
            Constants.UltrasonicConstants.PING_CHANNEL,
            Constants.UltrasonicConstants.ECHO_CHANNEL,
        )
     */
    private val CAN = Constants.ModuleConstants.CANBUS
    private val leftLaunchMotor =
        TalonFX(
            LaunchConstants.LEFT_LAUNCHMOTOR_ID,
            CAN,
        )
    private val rightLaunchMotor =
        TalonFX(
            LaunchConstants.RIGHT_LAUNCHMOTOR_ID,
            CAN,
        )

    private var targetRps = 0.0

    init {
        Ultrasonic.setAutomaticMode(true)
        val launchMotorConfig1 =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = LaunchConstants.LAUNCH_MOTOR_STATOR_LIMITS
                }

                Slot0.apply {
                    kP = LaunchConstants.LAUNCH_P
                    kI = LaunchConstants.LAUNCH_I
                    kD = LaunchConstants.LAUNCH_D
                    kS = LaunchConstants.LAUNCH_FF
                    kV = LaunchConstants.LAUNCH_V
                    kA = LaunchConstants.LAUNCH_A
                }

                MotorOutput.apply {
                    InvertedValue.Clockwise_Positive
                }
            }

        leftLaunchMotor.configurator.apply(launchMotorConfig1)

        val launchMotorConfig2 =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = LaunchConstants.LAUNCH_MOTOR_STATOR_LIMITS
                }

                Slot0.apply {
                    kP = LaunchConstants.LAUNCH_P
                    kI = LaunchConstants.LAUNCH_I
                    kD = LaunchConstants.LAUNCH_D
                    kS = LaunchConstants.LAUNCH_FF
                    kV = LaunchConstants.LAUNCH_V
                    kA = LaunchConstants.LAUNCH_A
                }

                MotorOutput.apply {
                    InvertedValue.CounterClockwise_Positive
                }
            }

        rightLaunchMotor.configurator.apply(launchMotorConfig2)
        // rangeFinder.isEnabled = true

//        val launchMotorConfig =
//            SparkMaxConfig().apply {
//                idleMode(SparkBaseConfig.IdleMode.kBrake)
//            }
//
//        launchMotor.configure(
//            launchMotorConfig,
//            SparkBase.ResetMode.kResetSafeParameters,
//            SparkBase.PersistMode.kPersistParameters,
//        )
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)

        builder.apply {
            addDoubleProperty("left motor voltage", {
                leftLaunchMotor.motorVoltage.valueAsDouble
            }, null)
            addDoubleProperty("right motor voltage", {
                rightLaunchMotor.motorVoltage.valueAsDouble
            }, null)
            addDoubleProperty("left stator current", {
                leftLaunchMotor.statorCurrent.valueAsDouble.takeIf { it.absoluteValue >= Constants.TelemetryConstants.MOTOR_CURRENT_MIN_RANGE } ?: 0.0
            }, null)
            addDoubleProperty("right stator current", {
                rightLaunchMotor.statorCurrent.valueAsDouble.takeIf { it.absoluteValue >= Constants.TelemetryConstants.MOTOR_CURRENT_MIN_RANGE } ?: 0.0
            }, null)
            addDoubleProperty("flywheel rps", { velocityRps() }, null)
            addDoubleProperty("flywheel target rps", { targetRps }, null)
        }
    }

    fun forward(launchProp: Double) {
        // leftLaunchMotor.setControl(VelocityVoltage(launchProp))
        // rightLaunchMotor.setControl(VelocityVoltage(launchProp))
        leftLaunchMotor.set(launchProp)
        rightLaunchMotor.set(-launchProp)
    }

    /**
     * Closed-loop flywheel speed in motor rotations per second (same sign convention as [forward]).
     * Unlike forward(), this holds the same speed whatever the battery voltage is, which is what makes
     * a distance -> speed table repeatable.
     */
    fun setVelocity(rotationsPerSecond: Double) {
        targetRps = rotationsPerSecond
        leftLaunchMotor.setControl(VelocityVoltage(rotationsPerSecond))
        rightLaunchMotor.setControl(VelocityVoltage(-rotationsPerSecond))
    }

    /** Average flywheel speed (rotations/s), positive in the shooting direction. */
    fun velocityRps(): Double = (leftLaunchMotor.velocity.valueAsDouble - rightLaunchMotor.velocity.valueAsDouble) / 2.0

    fun isAtVelocity(
        rotationsPerSecond: Double,
        toleranceRps: Double,
    ): Boolean = (velocityRps() - rotationsPerSecond).absoluteValue <= toleranceRps

    fun stop() {
        targetRps = 0.0
        leftLaunchMotor.stopMotor()
        rightLaunchMotor.stopMotor()
    }

    // Desktop simulation only: a physics model of the flywheel so velocity control, spin-up time and
    // the "at speed" check behave like the real thing. Both Krakens drive the same wheel.
    private val flywheelSim by lazy {
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), SimConstants.FLYWHEEL_MOI, 1.0),
            DCMotor.getKrakenX60(2),
        )
    }

    override fun simulationPeriodic() {
        val left = leftLaunchMotor.simState
        val right = rightLaunchMotor.simState
        left.setSupplyVoltage(RobotController.getBatteryVoltage())
        right.setSupplyVoltage(RobotController.getBatteryVoltage())

        // The right motor is commanded opposite, so subtract it to get the shared drive voltage.
        flywheelSim.setInputVoltage((left.motorVoltage - right.motorVoltage) / 2)
        flywheelSim.update(SimConstants.LOOP_SEC)

        val rps = Units.radiansToRotations(flywheelSim.angularVelocityRadPerSec)
        left.setRotorVelocity(rps)
        right.setRotorVelocity(-rps)
    }

    /*
    fun fuelInside(): Boolean =
    rangeFinder.rangeInches <
    Constants.UltrasonicConstants.CORAL_DISTANCE
     */
}
