package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

object StorageSubsystem : SubsystemBase() {
    private val rollerMotor = SparkMax(Constants.RollerConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val flyMotor = SparkMax(Constants.FlyConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    init {


        val rollerMotorConfig = SparkMaxConfig().apply{
            idleMode(SparkBaseConfig.IdleMode.kBrake)
        }

        rollerMotor.configure(
            rollerMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
    }

    fun roll(Rollerspeed: Double) {
        rollerMotor.set(Rollerspeed)
    }

    fun flywheel(Flywheelspeed: Double) {
        flyMotor.set(Flywheelspeed)
    }

    fun rollerstop() {
        rollerMotor.stopMotor()
    }

    fun flywheelstop(){
        flyMotor.stopMotor()
    }
}