/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.struct.Pose3dStruct
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator
import kotlinx.coroutines.Runnable

object Logger : SubsystemBase() {
    private val table = NetworkTableInstance.getDefault()
    private val field = Field2d()
    private val swerveType = SendableChooser<Constants.SomeConstants.SwerveType>()

    private val swervePublisher: StructArrayPublisher<SwerveModuleState?> =
        table
            .getStructArrayTopic("MyStates", SwerveModuleState.struct)
            .publish()
    private val limeLightPublisher: StructArrayPublisher<Pose3d?> =
        table
            .getStructArrayTopic("limeLight pose", Pose3d.struct)
            .publish()

    init {
        initTunablePID("DrivePID", PIDController(1.0, 0.0, 0.0))
        initTunablePID("TurnPID", PIDController(1.0, 0.0, 0.0))

        for (type in Constants.SomeConstants.SwerveType.entries) {
            swerveType.addOption(type.name, type)
        }
        SmartDashboard.putData("Swerve Type Chooser", swerveType)

        SmartDashboard.putData("Field", field)
        field.robotPose = Pose2d(Translation2d.kZero, Rotation2d.kZero)

        SmartDashboard.putData(
            "Reset Robot Pose",
            object : Command() {
                override fun execute() {
                    DriveSubsystem.resetPose(Pose2d.kZero)
                }

                override fun isFinished(): Boolean = true
            },
        )
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
    }

    fun initTunablePID(
        key: String,
        pid: PIDController,
    ) {
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putData(key, pid)
            SmartDashboard.setPersistent(key)
        }
    }

    fun safeGetData(key: String): Any {
//        while (!SmartDashboard.containsKey(key)) {
//            SmartDashboard.updateValues()
//            println("waiting for $key to arrive...")
//        }
//        return SmartDashboard.getData(key)

        while (true) {
            try {
                SmartDashboard.getData(key)
                break
            } catch (e: IllegalArgumentException) {
                continue
            }
        }
        return SmartDashboard.getData(key)
    }
}
