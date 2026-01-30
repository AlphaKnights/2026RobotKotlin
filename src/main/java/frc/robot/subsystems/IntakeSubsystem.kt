package frc.robot.subsystems

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

//HOPEFULLY this works

object IntakeSubsystem : SubsystemBase() {
    private val intakeMotor =
        SparkMax(
            Constants.IntakeConstants.MOTOR_ID,
            SparkLowLevel.MotorType.kBrushless,
        )
    init {
        val intakeMotorConfig =
            SparkMaxConfig().apply {
                idleMode(SparkBaseConfig.IdleMode.kBrake)
            }
        intakeMotor.configure(
            intakeMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters,
        )
    }
    fun forward(intakeProp: Double) {
        intakeMotor.set(intakeProp)
    }
    fun stop() {
        intakeMotor.stopMotor()
    }



}







