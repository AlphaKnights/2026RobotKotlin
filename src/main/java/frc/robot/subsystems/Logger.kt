/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEvent
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.reflect.KProperty

/**
Only use Logger for telemetry, not inputs or choosers!
 */
object Logger : SubsystemBase() {
    private val table = NetworkTableInstance.getDefault()
    private val field = Field2d()

    private val swervePublisher: StructArrayPublisher<SwerveModuleState?> =
        table
            .getStructArrayTopic("MyStates", SwerveModuleState.struct)
            .publish()
    private val limeLightPublisher: StructArrayPublisher<Pose3d?> =
        table
            .getStructArrayTopic("limeLight pose", Pose3d.struct)
            .publish()

    init {
        SmartDashboard.putData("Field", field)
        field.robotPose = Pose2d(Translation2d.kZero, Rotation2d.kZero)
    }

    override fun periodic() {
        field.robotPose = DriveSubsystem.getPose()
        field.getObject("targetPose").pose = DriveToArcPoseGenerator.generatePath()
    }

    /** Initializes all logged data.
     *
     *  Values are updated with SmartDashboard.updateValues()
     */
    fun initLog() {
        SmartDashboard.putNumber(
            "Match Time",
            DriverStation.getMatchTime(),
        )
        SmartDashboard.putNumber(
            "CAN Utilization",
            RobotController.getCANStatus().percentBusUtilization * 100,
        )
        SmartDashboard.putBoolean(
            "Tag Detected",
            LimelightSubsystem.tagPose != null,
        )
        SmartDashboard.putBoolean(
            "Aligned to Tag",
            LimelightSubsystem.isAligned(),
        )
//        SmartDashboard.putNumber(
//            "limelight x",
//            LimelightSubsystem.tagPose?.x ?: -1.0,
//        )
//        SmartDashboard.putNumber(
//            "limelight z",
//            LimelightSubsystem.tagPose?.z ?: -1.0,
//        )
//        SmartDashboard.putNumber(
//            "limelight yaw",
//            LimelightSubsystem.tagPose?.rotation?.y ?: -1.0,
//        )
        limeLightPublisher.set(arrayOf(LimelightSubsystem.tagPose))

        SmartDashboard.putNumber(
            "Shooting Distance",
            AimingCalc.canShoot(DriveSubsystem.getPose()),
        )
        SmartDashboard.putData("Field", field)

        swervePublisher.set(DriveSubsystem.getStates())

        SmartDashboard.putData(
            "Reset Robot Pose",
            object : Command() {
                override fun execute() {
                    DriveSubsystem.resetPose(Pose2d.kZero)
                }

                override fun isFinished(): Boolean = true
            },
        )
        SmartDashboard.putData("DriveSubsystem", DriveSubsystem)
    }

//    fun initTunablePID(key: String) {
//        val table =
//            NetworkTableInstance
//                .getDefault()
//                .getTable("SmartDashboard")
//                .getSubTable(key)
//
// //        val controller =
// //            PIDController(
// //                table.getEntry("p").getDouble(0.0),
// //                table.getEntry("i").getDouble(0.0),
// //                table.getEntry("d").getDouble(0.0),
// //            )
//        if (!table.subTables.contains(key)) {}
//        SmartDashboard.putData(key, PIDController(0.0, 0.0, 0.0))
//
//        for (name in listOf("p", "i", "d")) {
//            val entry = table.getEntry(name)
//            if (entry.exists()) {
//                entry.setPersistent()
//            }
//        }
//    }

    class TunablePIDController(
        key: String,
        kp: Double = 0.0,
        ki: Double = 0.0,
        kd: Double = 0.0,
    ) : PIDController(kp, ki, kd) {
        val controller = PIDController(kp, ki, kd)

        init {

            SmartDashboard.putData(key, controller)

            val table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(key)

            for (name in listOf("p", "i", "d")) {
                val entry = table.getEntry(name)
                if (entry.exists()) {
                    entry.setPersistent()
                }
            }
        }
    }

    fun PIDController.makeTunable(key: String): PIDController {
        val table = NetworkTableInstance.getDefault().getTable("SmartDashboard")
        SmartDashboard.putData(key, this)

        return this
    }
}

//    class TunablePID : Sendable {
//        fun set(name: String): DoubleSupplier {
//            return
//        }
//
//        fun get(name: String): DoubleConsumer {
//
//        }
//
//        override fun initSendable(builder: SendableBuilder?) {
//            builder.addDoubleProperty("kP", (this::set)("kP"), (this::get)("kP"))
//        }
//    }
