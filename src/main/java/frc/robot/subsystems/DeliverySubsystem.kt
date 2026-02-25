package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
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
    private val launchMotor =
        TalonFX(
            LaunchConstants.MOTOR_ID,
        )
    init {
        Ultrasonic.setAutomaticMode(true)

        val launchMotorConfig =
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
            }
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
        launchMotor.setControl(VelocityVoltage(launchProp))
    }

    fun stop() {
        launchMotor.stopMotor()
    }

    /*
    fun fuelInside(): Boolean =
    rangeFinder.rangeInches <
    Constants.UltrasonicConstants.CORAL_DISTANCE
    */
}
