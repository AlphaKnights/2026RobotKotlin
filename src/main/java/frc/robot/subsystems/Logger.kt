package frc.robot.subsystems

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import java.io.File
import java.lang.reflect.Type

object Logger : SubsystemBase() {
    private val field = Field2d()
    private val swerveType = SendableChooser<Constants.SomeConstants.SwerveType>()

    init {
        initTunablePID("DrivePID", PIDController(1.0, 0.0, 0.0))
        initTunablePID("TurnPID", PIDController(1.0, 0.0, 0.0))
        val swerveList =
            buildList {
                Constants.SomeConstants.SwerveType.entries
                    .forEach { type -> add(type) }
            }
        for (type in swerveList) {
            swerveType.addOption(type.name, type)
        }
        SmartDashboard.putData("Swerve Type Chooser", swerveType)
        SmartDashboard.putData("Field", field)

        field.setRobotPose(Pose2d(Translation2d.kZero, Rotation2d.kZero))
    }

    override fun periodic() {
        field.setRobotPose(DriveSubsystem.getPose())

        field.getObject("targetPose").setPose(DriveToArcPoseGenerator.generatePath())
    }

    fun log() {
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
        SmartDashboard.putNumber(
            "limelight x",
            LimelightSubsystem.tagPose?.x ?: -1.0,
        )
        SmartDashboard.putNumber(
            "limelight z",
            LimelightSubsystem.tagPose?.z ?: -1.0,
        )
        SmartDashboard.putNumber(
            "limelight yaw",
            LimelightSubsystem.tagPose?.rotation?.y ?: -1.0,
        )
        SmartDashboard.putNumber(
            "Shooting Distance",
            AimingCalc.canShoot(DriveSubsystem.getPose()),
        )
        SmartDashboard.putData("Field", field)
        SmartDashboard.putNumber(
            "DriveSpeed",
            DriveSubsystem.drive.fr
                .getState()
                .speedMetersPerSecond,
        )
        // SmartDashboard.putData("states", DriveSubsystem.states.)

        DriveSubsystem.swervePublisher.set(DriveSubsystem.getStates())
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
        while (true) {
            try {
                SmartDashboard.getData(key)
                break
            } catch (e: IllegalArgumentException) {
                println("error getting $key, retrying...")
                continue
            }
        }

        return SmartDashboard.getData(key)
    }
}
