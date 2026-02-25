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
        const val DRIVER_CONTROLLER_PORT = 1
        const val DRIVE_DEADBAND = 0.4

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 9 //joystick
        const val ALIGN_RIGHT_BUTTON = 10 //joystick

        const val BUTTON_BOARD_PORT = 2
        const val ELEVATOR_UP_BUTTON = 8
        const val ELEVATOR_DOWN_BUTTON = 10

        const val ELEVATOR_LVL_1_BUTTON = 1
        const val ELEVATOR_LVL_2_BUTTON = 2
        const val ELEVATOR_LVL_3_BUTTON = 3
        const val ELEVATOR_LVL_4_BUTTON = 4

        const val INTAKE_BUTTON = 6
        const val DELIVERY_BUTTON = 9
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 12
        const val MAX_ANGULAR_SPEED = 20

        private val TRACK_WIDTH = Units.inchesToMeters(26.5)
        private val WHEEL_BASE = Units.inchesToMeters(26.5)

        private val MODULE_POSITIONS = arrayOf(
            Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
        )

        val DRIVE_KINEMATICS = SwerveDriveKinematics(*MODULE_POSITIONS)

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(-0.764892578125)
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.579833984375)
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.23)
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.367919921875)

        const val FRONT_LEFT_DRIVING_ID = 5
        const val REAR_LEFT_DRIVING_ID = 3
        const val FRONT_RIGHT_DRIVING_ID = 7
        const val REAR_RIGHT_DRIVING_ID = 1

        const val FRONT_LEFT_TURNING_ID = 6
        const val REAR_LEFT_TURNING_ID = 4
        const val FRONT_RIGHT_TURNING_ID = 8
        const val REAR_RIGHT_TURNING_ID = 2

        const val FRONT_LEFT_CANCODER_ID = 3
        const val REAR_LEFT_CANCODER_ID = 2
        const val FRONT_RIGHT_CANCODER_ID = 4
        const val REAR_RIGHT_CANCODER_ID = 1

    }

    object ModuleConstants {
        const val DRIVE_RATIO = 17.326202353

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
    }

    object ElevatorConstants {
        const val LEFT_MOTOR_CAN_ID = 4
        const val RIGHT_MOTOR_CAN_ID = 32

        const val P = 0.08
        const val I = 0.0
        const val D = 0.0

        val IDLE_MODE = SparkBaseConfig.IdleMode.kCoast

        const val FORWARD_SOFT_LIMIT = 100.0
        const val REVERSE_SOFT_LIMIT = -100.0

        const val CURRENT_LIMIT = 40

        const val MAX_ELEVATOR_SPEED = 3.0

        const val LVL_1_HEIGHT = 0.0
        const val LVL_2_HEIGHT = 47.0
        const val LVL_3_HEIGHT  = 76.0
        const val LVL_4_HEIGHT = 50.0

        const val POS_DEADZONE = 0.2
    }

    enum class ElevatorDirection {
        UP,
        DOWN,
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

    object UltrasonicConstants {
        const val CORAL_DISTANCE = 3
        const val PING_CHANNEL = 8
        const val ECHO_CHANNEL = 9
    }

    object LaunchConstants {
        const val MOTOR_ID = 30
        const val LAUNCH_SPEED = 0.6
        const val LAUNCH_P = 0.1
        const val LAUNCH_I = 0.0
        const val LAUNCH_D = 0.0
        const val LAUNCH_FF = 0.0
        const val LAUNCH_V = 0.0071
        const val LAUNCH_A = 0.0
    }
}



