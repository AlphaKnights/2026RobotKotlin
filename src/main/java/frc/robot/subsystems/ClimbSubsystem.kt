package frc.robot.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.DeviceIdentifier
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Constants
import frc.robot.Constants.ClimbConstants
import frc.robot.Constants.ModuleConstants


object ClimbSubsystem : SubsystemBase() {
    private val ClimbArm =
        TalonFX(
            ClimbConstants.ClimbArmID,
        )


    init {
        // val ClimbPIDController =


        val ClimbConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = ClimbConstants.CLIMB_CURRENT_LIMIT
                }

                SoftwareLimitSwitch.apply {
                    ForwardSoftLimitEnable
                    ReverseSoftLimitEnable

                    ForwardSoftLimitThreshold = ClimbConstants.FORWARD_SOFT_LIMIT
                    ReverseSoftLimitThreshold = ClimbConstants.REVERSE_SOFT_LIMIT
                }

                Slot0.apply {
                    kI = ClimbConstants.I
                    kP = ClimbConstants.P
                    kD = ClimbConstants.D
                }

                ResetMode.kResetSafeParameters
                PersistMode.kPersistParameters

                /*encoder.apply {
                    positionConversionFactor(1.0)
                    velocityConversionFactor(1.0)
                }

                closedLoop.apply {
                    feedbackSensor(
                        FeedbackSensor.kPrimaryEncoder, //ClosedLoopConfig.Feedback.Sensor
                    )
                    pid(
                        ClimbConstants.P,
                        ClimbConstants.I,
                        ClimbConstants.D,
                    )
                    outputRange(-1.0, 1.0)
                    positionWrappingEnabled(false)*/
            }

        ClimbArm.getConfigurator().apply(ClimbConfig)
    }


    /**
     * Sets the speed of the elevator motors.
     * @param speed The proportion speed to set the motors to, between -1.0 and 1.0.
     */
    fun move(speed: Double) {
        ClimbArm.set(speed)

    }

    /**
     * Sets the elevator motors to a specific position.
     * @param position The position to set the motors to, in rotations.
     */
    fun setPosition(position: Double) {
        var m_request = PositionDutyCycle(0.0).withSlot(0)

        ClimbArm.setControl(m_request.withPosition(position))
//        ClimbPIDController.setSetpoint(
//            position,
//            SparkBase.ControlType.kPosition,
//        )
//    }

        /**
         * Gets the current position of the elevator.
         * @return The current position of the elevator, in rotations.
         */


    }
    fun getPosition(): Double {
        return ClimbArm.getPosition().getValueAsDouble()
    }
    fun stop() {
        ClimbArm.stopMotor()
    }
}

