package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.IntakeConstants.INTAK


object IntakeSubsystem : SubsystemBase() {

    private val intakeMotor: TalonFX = TalonFX(1)

    private val rightleverMotor: TalonFX = TalonFX(2)


    private val leftLeverMotor: TalonFX = TalonFX(3)



    init {
        val config: TalonFXConfiguration = TalonFXConfiguration()
        val leaderMotor = rightleverMotor: TalonFX = TalonFX(3)
        val followerMotor = leftLeverMotor
        config.apply {

        }
    }


    fun runMotor() {
        intakeMotor.set(1.0)
    }

    fun stopMotor() {
        intakeMotor.set(0.0)
    }

    fun moveIntake(direction: Boolean) {
        if (direction) {
            rightleverMotor.set(1.0)
            leftLeverMotor.set(1.0)
        }
        else {
            rightleverMotor.set(0.0)
            leftLeverMotor.set(0.0)
        }
    }






}







