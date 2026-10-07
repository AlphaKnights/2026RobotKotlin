/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.traits.CommonDevice
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.Constants.ModuleConstants
import frc.robot.RobotContainer
import frc.robot.interfaces.SwerveModule
import kotlin.reflect.KProperty
import kotlin.reflect.full.createType
import kotlin.reflect.full.instanceParameter
import kotlin.reflect.full.valueParameters
import kotlin.reflect.jvm.javaGetter
import kotlin.reflect.jvm.isAccessible
import java.io.File

class SwerveModuleIOTalon(
    driveMotorId: Int,
    turnMotorId: Int,
    encoderId: Int,
    private val offset: Rotation2d,
) : SwerveModule,
    Sendable {
    private val driveMotor = TalonFX(driveMotorId)
    private val turnMotor = TalonFX(turnMotorId)
    private val encoder = CANcoder(encoderId)
    private var desiredState =
        SwerveModuleState(
            0.0,
            Rotation2d.fromRotations(
                encoder.position.valueAsDouble,
            ) +
                    offset,
        )

    init {
        val driveMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit =
                        ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit =
                        ModuleConstants.DRIVING_STATOR_CURRENT_LIMIT
                }

                Slot0.apply {
                    kP = ModuleConstants.DRIVING_P
                    kI = ModuleConstants.DRIVING_I
                    kD = ModuleConstants.DRIVING_D
                    kS = ModuleConstants.DRIVING_FF
                    kV = ModuleConstants.DRIVING_V
                    kA = ModuleConstants.DRIVING_A
                }

                OpenLoopRamps.apply {
                    DutyCycleOpenLoopRampPeriod = 0.0
                }

                ClosedLoopRamps.apply {
                    DutyCycleClosedLoopRampPeriod = 0.0
                }

                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }

                Feedback.apply {
                    SensorToMechanismRatio =
                        ModuleConstants.DRIVE_RATIO
                }
                TalonFXConfiguration().Audio.AllowMusicDurDisable
            }

        val turnMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit =
                        ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit =
                        ModuleConstants.TURNING_STATOR_CURRENT_LIMIT
                }

                Feedback.apply {
                    SensorToMechanismRatio = 1.0
                    FeedbackRemoteSensorID = encoderId
                    FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder
                }

                ClosedLoopGeneral.apply {
                    ContinuousWrap = true
                }

                Slot0.apply {
                    kP = ModuleConstants.TURNING_P
                    kI = ModuleConstants.TURNING_I
                    kD = ModuleConstants.TURNING_D
                    kS = ModuleConstants.TURNING_FF
                }

                OpenLoopRamps.apply {
                    DutyCycleOpenLoopRampPeriod = 0.0
                }

                ClosedLoopRamps.apply {
                    DutyCycleClosedLoopRampPeriod = 0.0
                }

                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                    Inverted =
                        InvertedValue.Clockwise_Positive
                }
                TalonFXConfiguration().Audio.AllowMusicDurDisable
            }

        driveMotor.getConfigurator().apply(driveMotorConfig)
        turnMotor.getConfigurator().apply(turnMotorConfig)

        driveMotor.setPosition(0.0)
        generateOrchestra()
    }

    override fun getPosition(): SwerveModulePosition =
        SwerveModulePosition(
            // driveMotor.rotor
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveMotor.position.valueAsDouble,
            Rotation2d.fromRotations(
                turnMotor.position.valueAsDouble,
            ) +
                    offset,
        )

    override fun getState(): SwerveModuleState =
        SwerveModuleState(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveMotor.velocity.valueAsDouble,
            Rotation2d.fromRotations(
                turnMotor.position.valueAsDouble,
            ) +
                    offset,
        )

    override fun setDesiredState(desiredState: SwerveModuleState) {
        val correctedState =
            SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle - offset,
            )
        correctedState.optimize(
            Rotation2d.fromRotations(
                encoder.position.valueAsDouble,
            ),
        )

        driveMotor.setControl(
            VelocityVoltage(
                correctedState.speedMetersPerSecond / ModuleConstants.WHEEL_CIRCUMFERENCE,
            ),
        )
        turnMotor.setControl(
            PositionVoltage(correctedState.angle.rotations),
        )

        this.desiredState = desiredState
    }

    override fun initSendable(builder: SendableBuilder?) {
        builder?.apply {
            addDoubleProperty("drive motor voltage", { driveMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("turn motor voltage", { turnMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("drive stator current", { driveMotor.statorCurrent.valueAsDouble }, null)
            addDoubleProperty("turn stator current", { turnMotor.statorCurrent.valueAsDouble }, null)
        }
    }

    fun generateOrchestra() {

//         val subsystemList =
//             buildList {
//                 File("src/main/java/frc/robot/subsystems").listFiles()?.filter {frc.robot.subsystems.(it.nameWithoutExtension)::class.members.forEach {
//                 it.returnType.classifier == TalonFX::class && it is KProperty<*>
//             }
// }
//             }        
//         val motorRefs: List<KCallable<*>> =
//             this::class.members.filter {
//                 it.returnType.classifier == TalonFX::class && it is KProperty<*>
//             }
//         motorRefs.forEach {it.isAccessible = true}
        
//         println(motorRefs)
//         if (motorRefs.isEmpty()) {
//             println("WARNING, NO MOTORS IN ORCHESTRA")
//         }
//         motorRefs.forEach {
//             val motor = it.call(this)
//             RobotContainer.orchestra.addInstrument(motor as CommonDevice?)
//             println("$motor has been added")
//         }

            RobotContainer.orchestra.apply {
                addInstrument(driveMotor)
                addInstrument(turnMotor)
            }
    }
}
