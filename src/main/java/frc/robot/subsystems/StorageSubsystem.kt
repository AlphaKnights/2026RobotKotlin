package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.RollerConstants

object StorageSubsystem : SubsystemBase() {
    private val rollerMotor = TalonFX(RollerConstants.MOTOR_ID)
    init {

        val rollerMotorConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMITS
            }
        }

        rollerMotor.getConfigurator().apply(rollerMotorConfig)
    }

    fun roll(rollerSpeed: Double) {
        rollerMotor.set(rollerSpeed)
    }

    fun rollerstop() {
        rollerMotor.stopMotor()
    }

}