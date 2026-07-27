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
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.PathPlannerConstants
import frc.robot.interfaces.SwerveModule
import frc.robot.subsystems.GyroSim.gyro

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

object RealDriveSubsystem : SubsystemBase(), IDriveSubsystem {
    data class Swerve(
        val fl: SwerveModule,
        val fr: SwerveModule,
        val bl: SwerveModule,
        val br: SwerveModule,
    )

    val swerveTypeChooser = SendableChooser<Constants.SomeConstants.SwerveType>()

    private var gyro: Pigeon2 = Pigeon2(DriveConstants.PIDGEON2_ID)
//    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    private val drive: Swerve
        get() =
            if (swerveTypeChooser as Constants.SomeConstants.SwerveType ==
                Constants.SomeConstants.SwerveType.TALON
            ) {
                // Real robot, instantiate hardware IO implementations
                // SwerveModuleIOTalon is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
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
            } else { // TODO: Add actual robert IDs from 2025KitBot code
                Swerve(
                    SwerveModuleIOSparkMAX(
                        DriveConstants.FRONT_LEFT_DRIVING_ID,
                        DriveConstants.FRONT_LEFT_TURNING_ID,
                        DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
                    ),
                    SwerveModuleIOSparkMAX(
                        DriveConstants.FRONT_RIGHT_DRIVING_ID,
                        DriveConstants.FRONT_RIGHT_TURNING_ID,
                        DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
                    ),
                    SwerveModuleIOSparkMAX(
                        DriveConstants.REAR_LEFT_DRIVING_ID,
                        DriveConstants.REAR_LEFT_TURNING_ID,
                        DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
                    ),
                    SwerveModuleIOSparkMAX(
                        DriveConstants.REAR_RIGHT_DRIVING_ID,
                        DriveConstants.REAR_RIGHT_TURNING_ID,
                        DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
                    ),
                )
            }

    private var odometry =
        SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            // gyro.rotation3d.x
            arrayOf(
                drive.fl.getPosition(),
                drive.fr.getPosition(),
                drive.bl.getPosition(),
                drive.br.getPosition(),
            ),
        )

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        // gyro.reset()
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()

        for (type in Constants.SomeConstants.SwerveType.entries) {
            swerveTypeChooser.addOption(type.name, type)
        }
        SmartDashboard.putData("Swerve Type Chooser", swerveTypeChooser)

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

        odometry.update(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            arrayOf(
                drive.fl.getPosition(),
                drive.fr.getPosition(),
                drive.bl.getPosition(),
                drive.br.getPosition(),
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
            drive.fl.getState(),
            drive.fr.getState(),
            drive.bl.getState(),
            drive.br.getState(),
        )

    override fun getPose(): Pose2d = odometry.poseMeters

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    override fun getCurrentSpeeds(): ChassisSpeeds =
        DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            drive.fl.getState(),
            drive.fr.getState(),
            drive.bl.getState(),
            drive.br.getState(),
        )

    override fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            // gyro.getRotation2d(),
            arrayOf(
                drive.fl.getPosition(),
                drive.fr.getPosition(),
                drive.bl.getPosition(),
                drive.br.getPosition(),
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

        if (fieldRelative) {
            swerveModuleStates =
                DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
                    ),
                ) // gyro.getRotation2d()
        }

        drive.fl.setDesiredState(swerveModuleStates[0])
        drive.fr.setDesiredState(swerveModuleStates[1])
        drive.bl.setDesiredState(swerveModuleStates[2])
        drive.br.setDesiredState(swerveModuleStates[3])
    }

    override fun setX() {
        drive.fl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
        drive.fr.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        drive.bl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        drive.br.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
    }

    override fun zeroHeading() {
        gyro.reset()
    }
}

object PhysicsSimDriveSubsystem : SubsystemBase(), IDriveSubsystem {
    data class Swerve(
        val fl: SwerveModule,
        val fr: SwerveModule,
        val bl: SwerveModule,
        val br: SwerveModule,
    )

    private var gyro = GyroSim.gyro

    private var m_speeds: ChassisSpeeds = ChassisSpeeds()

    private val drive: Swerve = Swerve(SwerveModuleSim(), SwerveModuleSim(), SwerveModuleSim(), SwerveModuleSim())

    private var odometry =
        SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d(gyro.angularPosition),
            arrayOf(
                drive.fl.getPosition(),
                drive.fr.getPosition(),
                drive.bl.getPosition(),
                drive.br.getPosition(),
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
                drive.fl.getPosition(),
                drive.fr.getPosition(),
                drive.bl.getPosition(),
                drive.br.getPosition(),
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
            drive.fl.getState(),
            drive.fr.getState(),
            drive.bl.getState(),
            drive.br.getState(),
        )

    override fun getPose(): Pose2d = odometry.poseMeters

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    override fun getCurrentSpeeds(): ChassisSpeeds =
        DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            drive.fl.getState(),
            drive.fr.getState(),
            drive.bl.getState(),
            drive.br.getState(),
        )

    override fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d(gyro.angularPosition),
            // gyro.getRotation2d(),
            arrayOf(
                drive.fl.getPosition(),
                drive.fr.getPosition(),
                drive.bl.getPosition(),
                drive.br.getPosition(),
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

        drive.fl.setDesiredState(swerveModuleStates[0])
        drive.fr.setDesiredState(swerveModuleStates[1])
        drive.bl.setDesiredState(swerveModuleStates[2])
        drive.br.setDesiredState(swerveModuleStates[3])
    }

    override fun setX() {
        drive.fl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
        drive.fr.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        drive.bl.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        drive.br.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
    }

    override fun zeroHeading() {
        gyro.setState(0.0, 0.0)
    }
}

object GyroSim : SubsystemBase() {
    val gyro = DCMotorSim(LinearSystemId.createDCMotorSystem(1.0, 2.0), DCMotor.getKrakenX60Foc(1))

    // TODO: Tune kV and kA
    init {
        // SmartDashboard.putNumber("Gyro Position", Rotation2d(gyro.angularPosition).rotations)
    }

    fun update(speeds: ChassisSpeeds) {
        gyro.inputVoltage = speeds.toTwist2d(0.20).dtheta
        gyro.update(0.20)
    }
}

object SimpleSimDriveSubsystem : SubsystemBase(), IDriveSubsystem {
    var m_pose: Pose2d = Pose2d.kZero
    var m_speeds: ChassisSpeeds = ChassisSpeeds()

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
        // println(m_pose.toString())
        // SmartDashboard.putNumber("dTheta", m_speeds.toTwist2d(0.20).dtheta)
    }

    override fun getPose(): Pose2d = m_pose

    override fun getStates(): Array<SwerveModuleState>? = null

    override fun getCurrentSpeeds(): ChassisSpeeds = m_speeds

    override fun resetOdometry(newPose: Pose2d) {
        m_pose = newPose
    }

    override fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    override fun drive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean,
    ) {
        m_speeds = speeds
        m_pose = m_pose.exp(speeds.toTwist2d(0.20))
    }

    override fun setX() {
        // Does nothing in the sim
    }

    override fun zeroHeading() {
        // Does nothing in the sim
    }
}

private val delegate by lazy {
    if (Constants.SomeConstants.isReal) {
        RealDriveSubsystem
    } else {
        PhysicsSimDriveSubsystem
    }
}

object DriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem by delegate as IDriveSubsystem
