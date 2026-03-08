package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.LaunchConstants

object DeliverySubsystem : SubsystemBase() {
    /*
    private val rangeFinder =
        Ultrasonic(
            Constants.UltrasonicConstants.PING_CHANNEL,
            Constants.UltrasonicConstants.ECHO_CHANNEL,
        )
     */
    /* Ultrasonic if we need one */
    private val launchMotor1 =
        TalonFX(
            LaunchConstants.MOTOR_ID1,
        )
    private val launchMotor2 =
        TalonFX(
            LaunchConstants.MOTOR_ID2,
        )
    init {
        Ultrasonic.setAutomaticMode(true)

        val launchMotorConfig1 =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS
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

        launchMotor1.getConfigurator().apply(launchMotorConfig1)

        val launchMotorConfig2 =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS
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

        launchMotor2.getConfigurator().apply(launchMotorConfig2)
        /* rangeFinder.isEnabled = true */

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

    fun forward(launchProp: Double) {
        launchMotor1.setControl(VelocityVoltage(launchProp))
        launchMotor2.setControl(VelocityVoltage(launchProp))
    }

    fun stop() {
        launchMotor1.stopMotor()
        launchMotor2.stopMotor()
    }

    /*
    fun fuelInside(): Boolean =
    rangeFinder.rangeInches <
    Constants.UltrasonicConstants.CORAL_DISTANCE
    */
}
