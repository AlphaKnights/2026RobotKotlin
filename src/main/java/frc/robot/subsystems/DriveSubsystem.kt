package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.DoublePublisher
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.XBoxController
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator


object DriveSubsystem : SubsystemBase()
{

    private val xBoxController = XBoxController()

    private var frontLeft: TalonSwerveModule = TalonSwerveModule(
        Constants.DriveConstants.FRONT_LEFT_DRIVING_ID,
        Constants.DriveConstants.FRONT_LEFT_TURNING_ID,
        Constants.DriveConstants.FRONT_LEFT_CANCODER_ID,
        Constants.DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET
    )
    private var frontRight: TalonSwerveModule = TalonSwerveModule(
        Constants.DriveConstants.FRONT_RIGHT_DRIVING_ID,
        Constants.DriveConstants.FRONT_RIGHT_TURNING_ID,
        Constants.DriveConstants.FRONT_RIGHT_CANCODER_ID,
        Constants.DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET
    )
    private var rearLeft: TalonSwerveModule = TalonSwerveModule(
        Constants.DriveConstants.REAR_LEFT_DRIVING_ID,
        Constants.DriveConstants.REAR_LEFT_TURNING_ID,
        Constants.DriveConstants.REAR_LEFT_CANCODER_ID,
        Constants.DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET
    )
    private var rearRight: TalonSwerveModule = TalonSwerveModule(
        Constants.DriveConstants.REAR_RIGHT_DRIVING_ID,
        Constants.DriveConstants.REAR_RIGHT_TURNING_ID,
        Constants.DriveConstants.REAR_RIGHT_CANCODER_ID,
        Constants.DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET
    )
    private var FrontRightEncoder = CANcoder(Constants.DriveConstants.FRONT_RIGHT_CANCODER_ID)
    private var fL : TalonFX = TalonFX(Constants.DriveConstants.FRONT_LEFT_DRIVING_ID)

    private var gyro: Pigeon2 = Pigeon2(20)
//    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    private var odometry: SwerveDriveOdometry

    private val config: RobotConfig = RobotConfig.fromGUISettings()



    private val states: Array<SwerveModuleState> = arrayOf(
        frontLeft.getState(),
        frontRight.getState(),
        rearLeft.getState(),
        rearRight.getState(),
    )
    var swervePublisher: StructArrayPublisher<SwerveModuleState> = NetworkTableInstance.getDefault()
        .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish()
    var currentPublisher: DoublePublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("Drive/StatorCurrent").publish()
    var counter = 0


    init {
        //gyro.reset()
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()

        odometry = SwerveDriveOdometry(
            Constants.DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),   //gyro.rotation3d.x
            arrayOf(
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition(),
                )
        )

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            { speeds: ChassisSpeeds, _: DriveFeedforwards ->
                drive(speeds, fieldRelative = false)
            },
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(17.0, 0.0, 0.0),
                1.0,
            ),
            config,
            this::shouldFlipPath,
            this


        )
    }

    override fun periodic()
    {
        counter++
        // This method will be called once per scheduler run
//        if (Robot.isAutonomous()) {

            odometry.update(
                Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
                arrayOf(
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition(),
                )
            )

            resetOdometry(LimelightSubsystem.getPose()?.toPose2d() ?: getPose())
            println(getPose())

            println("ArcPose = ${DriveToArcPoseGenerator.generatePath()}")
            println("AllianceRed = ${(DriverStation.getAlliance() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red}")
      //  if(counter % 5 ==0) {
            // swervePublisher.set(states);
            //currentPublisher.set(fL.getStatorCurrent().getValueAsDouble())
            //currentPublisher.set(fR.getStatorCurrent().getValueAsDouble())
    //    }


//        }

//        println("Front Right:"+ FrontRightEncoder.getVelocity())
//        println("Front Right Speed: "+frontRight.getState().speedMetersPerSecond)
//        println("Front Left Speed: "+frontLeft.getState().speedMetersPerSecond)
//        println("Back Right Speed: "+rearRight.getState().speedMetersPerSecond)
//        println("Back Left Speed: "+rearLeft.getState().speedMetersPerSecond)
//        println("Odometry:"+getPose())
//
//        println("angle:"+gyro.getYaw())
//        println(xBoxController.getRawAxis(0))
    }

    fun getPose(): Pose2d {
        return odometry.poseMeters
    }

    fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    fun getCurrentSpeeds(): ChassisSpeeds {
        return Constants.DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState(),
        )
    }

    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble), //gyro.getRotation2d(),
            arrayOf(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition(),
            ),
            pose,
        )
    }

    fun shouldFlipPath(): Boolean {
        return false // (DriverStation.getAlliance() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red
    }

    fun drive(
         speeds: ChassisSpeeds,
         fieldRelative: Boolean,
    ) {

        var swerveModuleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)

        if (fieldRelative){
            swerveModuleStates = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(gyro.yaw.valueAsDouble))) //gyro.getRotation2d()
        }
        
        frontLeft.setDesiredState(swerveModuleStates[0])
        frontRight.setDesiredState(swerveModuleStates[1])
        rearLeft.setDesiredState(swerveModuleStates[2])
        rearRight.setDesiredState(swerveModuleStates[3])
    }

    fun setX() {
        frontLeft.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0)
            )
        )
        frontRight.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0)
            )
        )
        rearLeft.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(-45.0)
            )
        )
        rearRight.setDesiredState(
            SwerveModuleState(
                0.0,
                Rotation2d.fromDegrees(45.0)
            )
        )
    }

    fun zeroHeading() {
        gyro.reset()
    }
}