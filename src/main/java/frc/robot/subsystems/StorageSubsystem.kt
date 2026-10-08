// Arti, Shawn, Vince
package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.RollerConstants

object StorageSubsystem : SubsystemBase() {
    val motor1: TalonFX = TalonFX(1)

    val motor2: TalonFX = TalonFX(2)

    init {
        val config1: TalonFXConfiguration = TalonFXConfiguration()

        config1.apply {
            MotorOutputConfigs().Inverted = InvertedValue.CounterClockwise_Positive
        }
        val config2: TalonFXConfiguration = TalonFXConfiguration()

        config2.apply {
            MotorOutputConfigs().Inverted = InvertedValue.Clockwise_Positive
        }

        motor1.
        motor2.

    }

    fun runStorage(speed: Double) {
        motor1.set(speed)
        motor2.set(speed)
    }

    fun stopStorage() {
        motor1.stopMotor()
        motor2.stopMotor()
    }
}