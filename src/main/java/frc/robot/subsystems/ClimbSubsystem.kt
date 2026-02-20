package frc.robot.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.NeutralModeValue
import com.pathplanner.lib.config.RobotConfig
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


object ClimbSubsystem : SubsystemBase() {
    private val LeftArm =
        SparkMax(
            Constants.ClimbConstants.LeftArmID,
            SparkLowLevel.MotorType.kBrushless
        )


    private val LeftPIDController = LeftArm.closedLoopController

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        val commonConfigs = TalonFXConfiguration()
            .withMotorOutput(
                MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(Constants.ClimbConstants.BOB))
                    .withStatorCurrentLimitEnable(true)
            )
        val talonFXConfigurator: Unit = m_talonFX.getConfigurator()
        val motorConfigs = MotorOutputConfigs()
    }
    /**
     * Sets the speed of the elevator motors.
     * @param speed The proportion speed to set the motors to, between -1.0 and 1.0.
     */
    fun move(speed: Double) {
        LeftArm.set(speed)

    }

    /**
     * Sets the elevator motors to a specific position.
     * @param position The position to set the motors to, in rotations.
     */
    fun setPosition(position: Double) {
        LeftPIDController.setSetpoint(
            position,
            SparkBase.ControlType.kPosition,
        )
    }

    /**
     * Gets the current position of the elevator.
     * @return The current position of the elevator, in rotations.
     */
    fun getPosition(): Double = (LeftArm.encoder.position)

    fun stop() {
        LeftArm.stopMotor()
    }



}