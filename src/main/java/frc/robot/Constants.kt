package frc.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object Constants
{
    object OperatorConstants
    {
        const val DRIVER_CONTROLLER_PORT = 0
        const val DRIVE_DEADBAND = 0.01
        const val LERP_VAL = 0.035

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 9 //joystick
        const val ALIGN_RIGHT_BUTTON = 10 //joystick

        const val BUTTON_BOARD_PORT = 2

        const val ANGLE_BUTTON = 12 //joystick

        const val INTAKE_BUTTON = 11
        const val INTAKE_REVERSE_BUTTON = 4
        const val INTAKE_LEVER_OUT_MANUAL_BUTTON = 12
        const val INTAKE_LEVER_IN_MANUAL_BUTTON = 9
        const val INTAKE_LEVER_OUT_AUTO_BUTTON = 2
        const val INTAKE_LEVER_IN_AUTO_BUTTON = 1
        const val DELIVERY_BUTTON = 10
    }
    object IntakeConstants {
        const val INTAKE_MOTOR_ID = 34
        const val RIGHT_LEVER_MOTOR_ID = 31
        const val LEFT_LEVER_MOTOR_ID = 62
        const val INTAKE_SPEED = 0.5

        // Limits should be in rotations
        const val LEVER_LIMIT_FORWARD = 0.0
        const val LEVER_LIMIT_REVERSE = 0.25

        const val P = 0.08
        const val I = 0.0
        const val D = 0.0

        const val LEVER_OUT_POSITION = 0.25
        const val LEVER_IN_POSITION = 0.0

        const val LEVER_SPEED = 0.2 // in rpm

        const val INTAKE_CURRENT_LIMIT = 40.0

    }

    enum class IntakeDirection {
        IN,
        OUT,
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 5
        const val MAX_ANGULAR_SPEED = 5

        private val TRACK_WIDTH = Units.inchesToMeters(25.5)
        private val WHEEL_BASE = Units.inchesToMeters(25.5)

        private val MODULE_POSITIONS = arrayOf(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        )
        val DRIVE_KINEMATICS = SwerveDriveKinematics(*MODULE_POSITIONS)

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.83)
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.619)
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.817) //+ is clockwise
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.50)//- counter-clockwise


     //   back right - > front left
     //   back left - >front right
     //   front left -> back right
     // front right -> back left


        const val FRONT_LEFT_DRIVING_ID = 8 //8->4
        const val REAR_LEFT_DRIVING_ID = 7 //5->19
        const val FRONT_RIGHT_DRIVING_ID = 19 //19->5
        const val REAR_RIGHT_DRIVING_ID = 4  //4->8

        const val FRONT_LEFT_TURNING_ID = 2 //2->3
        const val REAR_LEFT_TURNING_ID = 5   //7->6
        const val FRONT_RIGHT_TURNING_ID = 6 //6->7
        const val REAR_RIGHT_TURNING_ID = 3  //3->2

        const val FRONT_LEFT_CANCODER_ID = 11   //11->9
        const val REAR_LEFT_CANCODER_ID = 12   //12->10
        const val FRONT_RIGHT_CANCODER_ID = 10  //10->12
        const val REAR_RIGHT_CANCODER_ID = 9    //9->11

    }

    object ModuleConstants {
        const val DRIVE_RATIO = 5.36
        val WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0)*PI

        //const val WHEEL_CIRCUMFERENCE = 0.5 // meters

        const val DRIVING_P = 0.8
        const val DRIVING_I = 0.0
        const val DRIVING_D = 0.0
        const val DRIVING_FF = 1.0
        const val DRIVING_V = 0.12//0.12*DRIVE_RATIO
        const val DRIVING_A = 1.5
        const val TURNING_P = 40.0
        const val TURNING_I = 0.0
        const val TURNING_D = 0.0
        const val TURNING_FF = 0.0

        const val DRIVING_MOTOR_CURRENT_LIMIT = 30.0
        const val TURNING_MOTOR_CURRENT_LIMIT = 30.0
        const val DRIVING_STATOR_CURRENT_LIMIT = 50.0
        const val TURNING_STATOR_CURRENT_LIMIT = 50.0
    }

    object LimelightConstants {
        const val POLLING_RATE = 20L
        const val TIMEOUT = 500L // milliseconds
                const val IP_ADDR = "10.66.95.200"
//        const val IP_ADDR = "172.29.0.1"
    }
    object LaunchConstants {
        const val LEFT_LAUNCHMOTOR_ID = 54
        const val RIGHT_LAUNCHMOTOR_ID = 28
        const val LAUNCH_SPEED = 0.85
        const val LAUNCH_P = 0.1
        const val LAUNCH_I = 0.0
        const val LAUNCH_D = 0.0
        const val LAUNCH_FF = 0.0
        const val LAUNCH_V = 0.0071
        const val LAUNCH_A = 0.0
        const val LAUNCH_MOTOR_CURRENT_LIMITS = 60.0
    }
    object RollerConstants {
        const val ROLLER_MOTOR_ID = 23
        const val ROLLER_SPEED = 0.6
        const val ROLLER_MOTOR_CURRENT_LIMITS = 40.0
        const val BUTTON = 3
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
    object AimingConstants {
        const val DISTANCE = 1.0
        const val GOOD_DISTANCE_TOLERANCE = 0.5
        const val MIDDLING_DISTANCE_TOLERANCE = 1.0

        const val HUB_X = 5.0
        const val HUB_Y = 5.0

        const val MAX_SPEED = 1.0
        const val SLOW_DISTANCE = 1.0
        const val MIN_SPEED = 0.2
    }
}



