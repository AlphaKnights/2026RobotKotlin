// Intake Hunter Reilly, Michael Zhang, Ricardo Dominguez

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
import frc.robot.Constants.IntakeConstants
import frc.robot.Constants.IntakeConstants.INTAKE_SPEED


object IntakeSubsystem : SubsystemBase() {

    private val intakeMotor: TalonFX = TalonFX(IntakeConstants.INTAKE_MOTOR_ID)

    private val rightleverMotor: TalonFX = TalonFX(IntakeConstants.RIGHT_LEVER_MOTOR_ID)


    private val leftLeverMotor: TalonFX = TalonFX(IntakeConstants.LEFT_LEVER_MOTOR_ID)



    init {
        val config: TalonFXConfiguration = TalonFXConfiguration()
        val leaderMotor = rightleverMotor: TalonFX = TalonFX(3)
        val followerMotor = leftLeverMotor
        config.apply {

        }
    }


    fun runMotor() {
        intakeMotor.set(IntakeConstants.INTAKE_SPEED)
    }

    fun stopMotor() {
        intakeMotor.stopMotor()
    }

    fun moveIntake(direction: Boolean) {
        if (direction) {
            rightleverMotor.set()
            leftLeverMotor.set()
        }
        else {
            rightleverMotor.set()
            leftLeverMotor.set(0.1)
        }
    }






}







