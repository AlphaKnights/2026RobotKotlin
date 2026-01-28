package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

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
        SparkMax(
            Constants.LaunchConstants.MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )
    init {
        Ultrasonic.setAutomaticMode(true)
        /* rangeFinder.isEnabled = true */

        val launchMotorConfig =
            SparkMaxConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kBrake)
            }

        launchMotor.configure(
            launchMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
    }

    fun forward(launchProp: Double) {
        launchMotor.set(launchProp)
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
