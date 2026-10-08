// Liam H and // Inesh H

package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.Ultrasonic
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.LaunchConstants
import frc.robot.Constants
import frc.robot.subsystems.DeliverySubsystem.motor
import frc.robot.subsystems.DeliverySubsystem.motor2

object DeliverySubsystem : SubsystemBase() {
    val motor: TalonFX = TalonFX(5)
    val motor2: TalonFX = TalonFX(6)

    init {
        val config: TalonFXConfiguration = TalonFXConfiguration()

        config.apply {     // sets the limit of the supply
            CurrentLimits.apply {SupplyCurrentLimitEnable = true  // sets the limit of the supply
                SupplyCurrentLimit = Constants.LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS}
            fun runDelivery() {

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


        }



    }

    fun runDelivery() {
        motor.set(1.0) //turn on motor
        motor2.set(1.0)

    }

    fun stop() {
        motor.set(0.0) //turn off motor
        motor.stopMotor()
        motor2.set(0.0)
        motor2.stopMotor()
    }


    }