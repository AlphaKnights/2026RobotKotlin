/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.FieldConstants
import frc.robot.Constants.PathPlannerConstants
import frc.robot.Constants.SimConstants
import frc.robot.Robot
import frc.robot.interfaces.SwerveModule
import frc.robot.subsystems.vision.LimelightVision
import kotlin.math.abs

interface IDriveSubsystem {
    fun getPose(): Pose2d

    fun getStates(): Array<SwerveModuleState>?

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    fun getCurrentSpeeds(): ChassisSpeeds

    fun resetOdometry(pose: Pose2d)

    fun resetPose(pose: Pose2d)

    fun shouldFlipPath(): Boolean =
        (DriverStation.getAlliance().get() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red

    fun drive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean,
    )

    fun setX()

    fun zeroHeading()
}

class RealDriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem {
    data class Swerve(
        val fl: SwerveModule,
        val fr: SwerveModule,
        val bl: SwerveModule,
        val br: SwerveModule,
    )

    val swerveTypeChooser = SendableChooser<Constants.SomeConstants.SwerveType>()

    private var gyro: Pigeon2 = Pigeon2(DriveConstants.PIDGEON2_ID)
//    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    private val driveTrain: Swerve =
        Swerve(
            SwerveModuleIOTalon(
                DriveConstants.FRONT_LEFT_DRIVING_ID,
                DriveConstants.FRONT_LEFT_TURNING_ID,
                DriveConstants.FRONT_LEFT_CANCODER_ID,
                DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
            ),
            SwerveModuleIOTalon(
                DriveConstants.FRONT_RIGHT_DRIVING_ID,
                DriveConstants.FRONT_RIGHT_TURNING_ID,
                DriveConstants.FRONT_RIGHT_CANCODER_ID,
                DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
            ),
            SwerveModuleIOTalon(
                DriveConstants.REAR_LEFT_DRIVING_ID,
                DriveConstants.REAR_LEFT_TURNING_ID,
                DriveConstants.REAR_LEFT_CANCODER_ID,
                DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
            ),
            SwerveModuleIOTalon(
                DriveConstants.REAR_RIGHT_DRIVING_ID,
                DriveConstants.REAR_RIGHT_TURNING_ID,
                DriveConstants.REAR_RIGHT_CANCODER_ID,
                DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
            ),
        )

    // Wheel odometry + gyro, corrected by Limelight AprilTag poses (see LimelightVision).
    // Everything is in the WPILib blue-origin field frame, the same frame PathPlanner uses.
    private val poseEstimator =
        SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            gyroRotation,
            modulePositions,
            Pose2d(),
        )

    // Field-relative driving uses the gyro relative to where the driver last pressed "reset heading"
    // (driver frame). The pose estimator keeps the true field heading separately, which aiming needs.
    private var driverYawOffsetDeg = 0.0

    // False until something that actually knows the field heading has set it: a PathPlanner auto
    // start pose, or a multi-tag Limelight solve while disabled. Until then we assume the robot was
    // zeroed facing away from our driver station.
    private var fieldHeadingTrusted = false

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        // gyro.reset()
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()
        poseEstimator.resetPosition(Rotation2d.kZero, modulePositions, Pose2d())

        SmartDashboard.putData("fl", driveTrain.fl)
        SmartDashboard.putData("fr", driveTrain.fr)
        SmartDashboard.putData("bl", driveTrain.bl)
        SmartDashboard.putData("br", driveTrain.br)

//        for (type in Constants.SomeConstants.SwerveType.entries) {
//            swerveTypeChooser.addOption(type.name, type)
//        }
//        SmartDashboard.putData("Swerve Type Chooser", swerveTypeChooser)
        AutoBuilder
            .configure(
                this::getPose,
                this::resetPose,
                this::getCurrentSpeeds,
                { speeds: ChassisSpeeds, _: DriveFeedforwards ->
                    drive(speeds, fieldRelative = false)
                },
                PPHolonomicDriveController(
                    PIDConstants(
                        PathPlannerConstants.TRANSLATION_P,
                        PathPlannerConstants.TRANSLATION_I,
                        PathPlannerConstants.TRANSLATION_D,
                    ),
                    PIDConstants(
                        PathPlannerConstants.ROTATION_P,
                        PathPlannerConstants.ROTATION_I,
                        PathPlannerConstants.ROTATION_D,
                    ),
                    1.0,
                ),
                config,
                this::shouldFlipPath,
                this,
            )
    }

//    override fun initSendable(builder: SendableBuilder?) {
//        super.initSendable(builder)
//
//
//        fun SendableBuilder.addMotorStats(vararg motors: Pair<String, TalonFX>) {
//            motors.forEach { m ->
//                val name = m.first
//                val motor = m.second
//                builder?.apply {
//                    addDoubleProperty("$name motor voltage", { motor.motorVoltage.valueAsDouble }, null)
//                    addDoubleProperty("$name stator current", { motor.statorCurrent.valueAsDouble }, null)
//                }
//            }
//        }
//
//        builder?.addMotorStats(drive.fl.)
//    }

    override fun periodic() {
        // This method will be called once per scheduler run
//        if (Robot.isAutonomous()) {

        poseEstimator.update(gyroRotation, modulePositions)

        if (!fieldHeadingTrusted) assumeZeroedFacingAwayFromDriverStation()

        // Vision only nudges the estimate (no more hard resets every frame). While disabled it may
        // hand back a multi-tag pose that also fixes the heading.
        LimelightVision
            .update(poseEstimator, gyro.angularVelocityZWorld.valueAsDouble, DriverStation.isDisabled())
            ?.let { resetPose(it) }

        SmartDashboard.putBoolean("Vision/FieldHeadingTrusted", fieldHeadingTrusted)
    }

    private val gyroRotation: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.yaw.valueAsDouble)

    private val driverRotation: Rotation2d
        get() = Rotation2d.fromDegrees(gyro.yaw.valueAsDouble - driverYawOffsetDeg)

    private val modulePositions: Array<SwerveModulePosition>
        get() =
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            )

    /**
     * Until vision or an auto tells us the real field heading, assume the driver frame lines up with
     * the field: heading-reset was pressed while facing away from our driver station.
     */
    private fun assumeZeroedFacingAwayFromDriverStation() {
        val forward = allianceForward() ?: return
        val assumed = forward.plus(driverRotation)
        val current = getPose()
        if (abs(MathUtil.angleModulus(assumed.radians - current.rotation.radians)) > HEADING_EPSILON_RAD) {
            poseEstimator.resetPosition(gyroRotation, modulePositions, Pose2d(current.translation, assumed))
        }
    }

    override fun getStates(): Array<SwerveModuleState> =
        arrayOf(
            driveTrain.fl.getState(),
            driveTrain.fr.getState(),
            driveTrain.bl.getState(),
            driveTrain.br.getState(),
        )

    override fun getPose(): Pose2d = poseEstimator.estimatedPosition

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    override fun getCurrentSpeeds(): ChassisSpeeds =
        DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            driveTrain.fl.getState(),
            driveTrain.fr.getState(),
            driveTrain.bl.getState(),
            driveTrain.br.getState(),
        )

    override fun resetOdometry(pose: Pose2d) {
        poseEstimator.resetPosition(gyroRotation, modulePositions, pose)
    }

    /** Used by PathPlanner at the start of an auto and by vision seeding: both know the real heading. */
    override fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
        fieldHeadingTrusted = true
    }

    override fun drive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean,
    ) {
        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)

        if (fieldRelative) {
            // Driver frame, exactly like before: forward = the way the robot faced at the last heading reset.
            swerveModuleStates =
                DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        driverRotation,
                    ),
                )
        }

        driveTrain.fl.setDesiredState(swerveModuleStates[0])
        driveTrain.fr.setDesiredState(swerveModuleStates[1])
        driveTrain.bl.setDesiredState(swerveModuleStates[2])
        driveTrain.br.setDesiredState(swerveModuleStates[3])
    }

    override fun setX() {
        driveTrain.fl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
        driveTrain.fr.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        driveTrain.bl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        driveTrain.br.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
    }

    /**
     * Driver "reset heading": the direction the robot faces now becomes driver-forward. Done in
     * software instead of resetting the Pigeon so the pose estimator's heading never jumps.
     */
    override fun zeroHeading() {
        driverYawOffsetDeg = gyro.yaw.valueAsDouble
    }

    private companion object {
        const val HEADING_EPSILON_RAD = 1e-3
    }
}

class PhysicsSimDriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem {
    data class Swerve(
        val fl: SwerveModule,
        val fr: SwerveModule,
        val bl: SwerveModule,
        val br: SwerveModule,
    )

    private var gyro = GyroSim.gyro

    private var m_speeds: ChassisSpeeds = ChassisSpeeds()

    private val driveTrain: Swerve = Swerve(SwerveModuleSim(), SwerveModuleSim(), SwerveModuleSim(), SwerveModuleSim())

    private var odometry =
        SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d(gyro.angularPosition),
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            ),
        )

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        // gyro.reset()
//        gyro.enableBoardlevelYawReset(false)

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            { speeds: ChassisSpeeds, _: DriveFeedforwards ->
                drive(speeds, fieldRelative = false)
            },
            PPHolonomicDriveController(
                PIDConstants(
                    PathPlannerConstants.TRANSLATION_P,
                    PathPlannerConstants.TRANSLATION_I,
                    PathPlannerConstants.TRANSLATION_D,
                ),
                PIDConstants(
                    PathPlannerConstants.ROTATION_P,
                    PathPlannerConstants.ROTATION_I,
                    PathPlannerConstants.ROTATION_D,
                ),
                1.0,
            ),
            config,
            this::shouldFlipPath,
            this,
        )
    }

    override fun periodic() {
        // This method will be called once per scheduler run
//        if (Robot.isAutonomous()) {

        GyroSim.update(m_speeds)

        odometry.update(
            Rotation2d(gyro.angularPosition),
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            ),
        )

        resetOdometry(
            LimelightSubsystem.getPose()?.toPose2d()?.relativeTo(
                Pose2d(PathPlannerConstants.FIELD_SIZE, Rotation2d()),
            ) ?: getPose(),
        ) // limelight synchronization
    }

    override fun getStates(): Array<SwerveModuleState> =
        arrayOf(
            driveTrain.fl.getState(),
            driveTrain.fr.getState(),
            driveTrain.bl.getState(),
            driveTrain.br.getState(),
        )

    override fun getPose(): Pose2d = odometry.poseMeters

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    override fun getCurrentSpeeds(): ChassisSpeeds =
        DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            driveTrain.fl.getState(),
            driveTrain.fr.getState(),
            driveTrain.bl.getState(),
            driveTrain.br.getState(),
        )

    override fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d(gyro.angularPosition),
            // gyro.getRotation2d(),
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            ),
            pose,
        )
    }

    override fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    override fun drive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean,
    ) {
        var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
        m_speeds = speeds
        if (fieldRelative) {
            swerveModuleStates =
                DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        Rotation2d(gyro.angularPosition),
                    ),
                )
        }

        driveTrain.fl.setDesiredState(swerveModuleStates[0])
        driveTrain.fr.setDesiredState(swerveModuleStates[1])
        driveTrain.bl.setDesiredState(swerveModuleStates[2])
        driveTrain.br.setDesiredState(swerveModuleStates[3])
    }

    override fun setX() {
        driveTrain.fl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
        driveTrain.fr.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        driveTrain.bl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        driveTrain.br.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
    }

    override fun zeroHeading() {
        gyro.setState(0.0, 0.0)
    }

    private object GyroSim {
        val gyro = DCMotorSim(LinearSystemId.createDCMotorSystem(1.0, 2.0), DCMotor.getKrakenX60Foc(1))

        // TODO: Tune kV and kA

        fun update(speeds: ChassisSpeeds) {
            gyro.inputVoltage = speeds.toTwist2d(0.20).dtheta
            gyro.update(0.20)
        }
    }
}

/**
 * Ideal swerve for desktop simulation (`./gradlew simulateJava`): the robot does what it is told,
 * limited only by acceleration. Good for testing commands, aiming, auto-align, bindings and autos.
 * It does not model wheel slip, the real drive motors, or the Limelight (the pose is simply exact).
 */
class SimpleSimDriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem {
    private var pose: Pose2d = startPose()
    private var commandedSpeeds = ChassisSpeeds() // robot-relative
    private var speeds = ChassisSpeeds() // robot-relative, after acceleration limits

    // Driver frame, like the real robot: forward = where the robot faced at the last heading reset.
    private var driverOffset: Rotation2d = pose.rotation
    private var placedForAlliance = false

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            { speeds: ChassisSpeeds, _: DriveFeedforwards ->
                drive(speeds, fieldRelative = false)
            },
            PPHolonomicDriveController(
                PIDConstants(
                    PathPlannerConstants.TRANSLATION_P,
                    PathPlannerConstants.TRANSLATION_I,
                    PathPlannerConstants.TRANSLATION_D,
                ),
                PIDConstants(
                    PathPlannerConstants.ROTATION_P,
                    PathPlannerConstants.ROTATION_I,
                    PathPlannerConstants.ROTATION_D,
                ),
                1.0,
            ),
            config,
            this::shouldFlipPath,
            this,
        )
    }

    override fun periodic() {
        // Until the sim is enabled the first time, sit in front of whichever HUB the sim DS alliance owns.
        if (!placedForAlliance && DriverStation.isDisabled()) {
            pose = startPose()
            driverOffset = pose.rotation
        }
        if (DriverStation.isEnabled()) placedForAlliance = true

        val dt = SimConstants.LOOP_SEC
        speeds =
            ChassisSpeeds(
                approach(speeds.vxMetersPerSecond, commandedSpeeds.vxMetersPerSecond, SimConstants.MAX_ACCEL * dt),
                approach(speeds.vyMetersPerSecond, commandedSpeeds.vyMetersPerSecond, SimConstants.MAX_ACCEL * dt),
                approach(
                    speeds.omegaRadiansPerSecond,
                    commandedSpeeds.omegaRadiansPerSecond,
                    SimConstants.MAX_ANGULAR_ACCEL * dt,
                ),
            )
        pose = pose.exp(speeds.toTwist2d(dt))
    }

    /** In front of our HUB facing it: blue numbers, mirrored for red. */
    private fun startPose(): Pose2d =
        if (DriverStation.getAlliance().map { it == DriverStation.Alliance.Red }.orElse(false)) {
            Pose2d(FieldConstants.FIELD_LENGTH - SimConstants.START_X_BLUE, FieldConstants.HUB_Y, Rotation2d.k180deg)
        } else {
            Pose2d(SimConstants.START_X_BLUE, FieldConstants.HUB_Y, Rotation2d.kZero)
        }

    override fun getPose(): Pose2d = pose

    override fun getStates(): Array<SwerveModuleState> = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)

    override fun getCurrentSpeeds(): ChassisSpeeds = speeds

    override fun resetOdometry(pose: Pose2d) {
        this.pose = pose
        placedForAlliance = true
    }

    override fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    override fun drive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean,
    ) {
        commandedSpeeds =
            if (fieldRelative) {
                ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.rotation.minus(driverOffset))
            } else {
                speeds
            }
    }

    override fun setX() {
        commandedSpeeds = ChassisSpeeds()
    }

    override fun zeroHeading() {
        driverOffset = pose.rotation
    }
}

/** Heading 0 = facing the red wall. "Away from our driver station" is 0 for blue, 180 for red. */
private fun allianceForward(): Rotation2d? =
    DriverStation
        .getAlliance()
        .map { if (it == DriverStation.Alliance.Red) Rotation2d.k180deg else Rotation2d.kZero }
        .orElse(null)

/** Move [current] toward [target] by at most [maxStep]. */
private fun approach(
    current: Double,
    target: Double,
    maxStep: Double,
): Double = current + (target - current).coerceIn(-maxStep, maxStep)

private val delegate: IDriveSubsystem by lazy {
    if (Robot.isReallyReal) {
        RealDriveSubsystem()
    } else {
        // Desktop simulation. PhysicsSimDriveSubsystem is the older, untuned motor-level sim.
        SimpleSimDriveSubsystem()
    }
}

object DriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem by delegate
