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
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator

/**
Only use Logger for telemetry, not inputs or choosers!
 */
object Telemetry : SubsystemBase() {
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
        SmartDashboard.putData("intake", IntakeSubsystem)
        SmartDashboard.putData("shooter", DeliverySubsystem)

        SmartDashboard.putNumber(
            "Intake Position",
            IntakeSubsystem.getPosition(),
        )
        SmartDashboard.putNumber(
            "Intake Error",
            IntakeSubsystem.rightleverMotor.closedLoopError.valueAsDouble,
        )

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

    override fun periodic() {
        // field.robotPose = DriveSubsystem.getPose()
        field.getObject("targetPose").pose = DriveToArcPoseGenerator.generatePath(field.robotPose)
    }

    fun PIDController.makeTunable(key: String): PIDController {
        val table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(key)
        // ex: SmartDashboard/DrivePID

        // set this PID controller to the NT persistent values
        this.setPID(
            table.getEntry("p").getDouble(0.0),
            table.getEntry("i").getDouble(0.0),
            table.getEntry("d").getDouble(0.0),
        )

        // if this is a new controller, make the values persistent (ignored otherwise)
        for (name in listOf("p", "i", "d")) {
            val entry = table.getEntry(name)
            if (entry.exists()) {
                entry.setPersistent()
            }
        }
        // Use the Sendable api to make a PIDController appear on the dashboard with our constants
        SmartDashboard.putData(key, this)

        // return the PIDController as tuned from Glass (or elsewhere)
        return SmartDashboard.getData(key) as PIDController
    }
}
