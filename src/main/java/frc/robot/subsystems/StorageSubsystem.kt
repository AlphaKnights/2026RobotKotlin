package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.RollerConstants

object StorageSubsystem : SubsystemBase() {
    private val CAN = CANBus("didy")
    private val rollerMotor =

    private val rollerMotor2 =

    init {

        val rollerMotorConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMITS
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = RollerConstants.ROLLER_MOTOR_STATOR_LIMITS
            }
        }

    }

}