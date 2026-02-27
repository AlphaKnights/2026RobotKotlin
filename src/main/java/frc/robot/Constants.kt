package frc.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

import com.revrobotics.spark.config.SparkBaseConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units

object Constants
{
    object OperatorConstants
    {
        const val DRIVER_CONTROLLER_PORT = 0
        const val DRIVE_DEADBAND = 0.01

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 9 //joystick
        const val ALIGN_RIGHT_BUTTON = 10 //joystick

        const val BUTTON_BOARD_PORT = 2
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 2
        const val MAX_ANGULAR_SPEED = 6

        private val TRACK_WIDTH = Units.inchesToMeters(25.5)
        private val WHEEL_BASE = Units.inchesToMeters(25.5)

        private val MODULE_POSITIONS = arrayOf(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        )

        val DRIVE_KINEMATICS = SwerveDriveKinematics(*MODULE_POSITIONS)

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.0)
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.317)
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.119) //+ is clockwise
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.33)//- counter-clockwise


     //   back right - > front left
     //   back left - >front right
     //   front left -> back right
     // front right -> back left


        const val FRONT_LEFT_DRIVING_ID = 4 //8->4
        const val REAR_LEFT_DRIVING_ID = 19  //5->19
        const val FRONT_RIGHT_DRIVING_ID = 7 //19->5
        const val REAR_RIGHT_DRIVING_ID = 8  //4->8

        const val FRONT_LEFT_TURNING_ID = 3 //2->3
        const val REAR_LEFT_TURNING_ID = 6   //7->6
        const val FRONT_RIGHT_TURNING_ID = 5 //6->7
        const val REAR_RIGHT_TURNING_ID = 2  //3->2

        const val FRONT_LEFT_CANCODER_ID = 9   //11->9
        const val REAR_LEFT_CANCODER_ID = 10   //12->10
        const val FRONT_RIGHT_CANCODER_ID = 12  //10->12
        const val REAR_RIGHT_CANCODER_ID = 11    //9->11

    }

    object ModuleConstants {
        const val DRIVE_RATIO = 5.36

        const val DRIVING_P = 0.8
        const val DRIVING_I = 0.0
        const val DRIVING_D = 0.0
        const val DRIVING_FF = 1.0
        const val DRIVING_V = 0.3
        const val DRIVING_A = 1.5
        const val TURNING_P = 40.0
        const val TURNING_I = 0.0
        const val TURNING_D = 0.0
        const val TURNING_FF = 0.0

        const val DRIVING_MOTOR_CURRENT_LIMIT = 40.0
        const val TURNING_MOTOR_CURRENT_LIMIT = 40.0
        const val DRIVING_STATOR_CURRENT_LIMIT = 30.0
        const val TURNING_STATOR_CURRENT_LIMIT = 30.0
    }

    object LimelightConstants {
        const val POLLING_RATE = 20L
        const val TIMEOUT = 500L // milliseconds
        //        const val IP_ADDR = "10.66.95.11"
        const val IP_ADDR = "172.29.0.1"
    }

    object AlignConstants {
        const val ALIGN_DEADZONE = 0.03
        val ALIGN_ROT_DEADZONE = Units.degreesToRadians(5.0)

        const val FINE_ALIGN_DEADZONE = 1.0
        val FINE_ALIGN_ROT_DEADZONE = Units.degreesToRadians(5.0)

        const val MAX_SPEED = 1.0
        const val MAX_ANGULAR_SPEED = 1.0

        const val LEFT_X_OFFSET = -0.1625
        const val LEFT_Z_OFFSET = 0.457

        const val RIGHT_X_OFFSET = 0.1625
        const val RIGHT_Z_OFFSET = 0.457

        const val ALIGN_TIMEOUT = 5 // seconds
        const val ALIGN_SEEK_TIMEOUT = 1 // seconds
    }

    enum class AlignDirection {
        LEFT,
        RIGHT,
    }
}



