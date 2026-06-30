package frc.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator

class Logger : SubsystemBase() {
    private val field = Field2d()

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
            RobotController.getCANStatus().percentBusUtilization *
                100,
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
    }
}
