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
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.PathPlannerConstants
import frc.robot.Constants.SomeConstants.Mode

object DriveSubsystem : SubsystemBase() {
    data class Swerve(
        val fl: SwerveModule,
        val fr: SwerveModule,
        val bl: SwerveModule,
        val br: SwerveModule,
    )

    val field = Logger.safeGetData("Field") as Field2d

    private var gyro: Pigeon2 = Pigeon2(Constants.DriveConstants.PIDGEON2_ID)
//    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    var drive: Swerve =
        when (Constants.SomeConstants.currentMode) {
            Constants.SomeConstants.Mode.REAL -> {
                if (Logger.safeGetData("Swerve Type Chooser") as Constants.SomeConstants.SwerveType ==
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
            }

            Constants.SomeConstants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                Swerve(
                    SwerveModuleSim(),
                    SwerveModuleSim(),
                    SwerveModuleSim(),
                    SwerveModuleSim(),
                )
            }
        }

    private var odometry: SwerveDriveOdometry

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    var swervePublisher: StructArrayPublisher<SwerveModuleState> =
        NetworkTableInstance
            .getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct)
            .publish()

    var counter = 0

    init {
        // gyro.reset()
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()

        odometry =
            SwerveDriveOdometry(
                Constants.DriveConstants.DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
                // gyro.rotation3d.x
                arrayOf(
                    drive.fl.getPosition(),
                    drive.fr.getPosition(),
                    drive.bl.getPosition(),
                    drive.br.getPosition(),
                ),
            )

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
        counter++
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

    fun getStates(): Array<SwerveModuleState> =
        arrayOf(
            drive.fl.getState(),
            drive.fr.getState(),
            drive.bl.getState(),
            drive.br.getState(),
        )

    fun getPose(): Pose2d = odometry.poseMeters

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    fun getCurrentSpeeds(): ChassisSpeeds =
        Constants.DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            drive.fl.getState(),
            drive.fr.getState(),
            drive.bl.getState(),
            drive.br.getState(),
        )

    fun resetOdometry(pose: Pose2d) {
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

    fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    fun shouldFlipPath(): Boolean =
        (DriverStation.getAlliance().get() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red

    fun drive(
        speeds: ChassisSpeeds,
        fieldRelative: Boolean,
    ) {
        var swerveModuleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)

        if (fieldRelative) {
            swerveModuleStates =
                Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
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

    fun setX() {
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

    fun zeroHeading() {
        gyro.reset()
    }
}
