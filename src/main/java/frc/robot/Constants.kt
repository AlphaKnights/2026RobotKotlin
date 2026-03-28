/*
 * (C) 2025 Galvaknights
 */
package frc.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import kotlin.math.PI

object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val DRIVE_DEADBAND = 0.01
        const val LERP_VAL = 0.035

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 9 // joystick
        const val ALIGN_RIGHT_BUTTON = 10 // joystick

        const val BUTTON_BOARD_PORT = 2

        const val ANGLE_BUTTON = 12 // joystick
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 3.0
        const val MAX_ANGULAR_SPEED = 6.0

        val PATH_CONSTRAINTS =
            PathConstraints(
                MAX_METERS_PER_SECOND,
                10.0,
                MAX_ANGULAR_SPEED,
                4 * Math.PI,
                )

        private val TRACK_WIDTH = Units.inchesToMeters(25.5)
        private val WHEEL_BASE = Units.inchesToMeters(25.5)

        private val MODULE_POSITIONS =
            arrayOf(
                Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            )

        val DRIVE_KINEMATICS = SwerveDriveKinematics(*MODULE_POSITIONS)

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.0)
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.317)
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(.119) // + is clockwise
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.33) // - counter-clockwise

        //   back right - > front left
        //   back left - >front right
        //   front left -> back right
        // front right -> back left

        const val FRONT_LEFT_DRIVING_ID = 4 // 8->4
        const val REAR_LEFT_DRIVING_ID = 19 // 5->19
        const val FRONT_RIGHT_DRIVING_ID = 7 // 19->5
        const val REAR_RIGHT_DRIVING_ID = 8 // 4->8

        const val FRONT_LEFT_TURNING_ID = 3 // 2->3
        const val REAR_LEFT_TURNING_ID = 6 // 7->6
        const val FRONT_RIGHT_TURNING_ID = 5 // 6->7
        const val REAR_RIGHT_TURNING_ID = 2 // 3->2

        const val FRONT_LEFT_CANCODER_ID = 9 // 11->9
        const val REAR_LEFT_CANCODER_ID = 10 // 12->10
        const val FRONT_RIGHT_CANCODER_ID = 12 // 10->12
        const val REAR_RIGHT_CANCODER_ID = 11 // 9->11
    }

    object ModuleConstants {
        const val DRIVE_RATIO = 5.36
        val WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * PI

        // const val WHEEL_CIRCUMFERENCE = 0.5 // meters

        const val DRIVING_P = 0.8
        const val DRIVING_I = 0.0
        const val DRIVING_D = 0.0
        const val DRIVING_FF = 1.0
        const val DRIVING_V = 0.12 // 0.12*DRIVE_RATIO
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

    object AlignConstants {
        const val ALIGN_DEADZONE = 0.03

        // ALIGN_ROT_DEADZONE: the robot stops rotating once it's within this angle of center.
        val ALIGN_ROT_DEADZONE: Double = Units.degreesToRadians(5.0)

        const val FINE_ALIGN_DEADZONE = 1.0

        // FINE_ALIGN_ROT_DEADZONE: the robot starts slowing its rotation within this wider zone.
        // Think of it like speed limit signs approaching a school zone:
        //
        //  Full speed  ←─── [> 20°] ───→  Slow down  ←─── [5° to 20°] ───→  Stop  ←─ [< 5°] ─→
        //
        // Bug fix: This was 5.0° — identical to ALIGN_ROT_DEADZONE. When the two thresholds
        // are the same value, the "slow down" zone has zero width: the robot goes from full
        // speed directly to stopped with no ramp in between. 20° gives a 15° ramp zone.
        @Suppress("MagicNumber")
        val FINE_ALIGN_ROT_DEADZONE: Double = Units.degreesToRadians(20.0)

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

        // Hub field positions (meters). Set to real field measurements before competition.
        // Red hub: robot approaches from y < RED_HUB_Y
        // Blue hub: robot approaches from y > BLUE_HUB_Y
        const val RED_HUB_X = 5.0
        const val RED_HUB_Y = 5.0
        const val BLUE_HUB_X = 5.0
        const val BLUE_HUB_Y = 3.0 // placeholder — team must tune

        // Valid shooting-arc sector, in degrees, measured from hub center.
        //   0° = +X on field,  90° = +Y,  180° = -X,  270° = -Y (toward driver station)
        // These values apply for red alliance (robot below hub, y < RED_HUB_Y).
        // For blue alliance the sector is mirrored vertically — team must tune.
        const val MIN_ANGLE_DEGREES = 200.0
        const val MAX_ANGLE_DEGREES = 340.0

        // Fallback when DriverStation hasn't reported an alliance yet (e.g. practice mode).
        // true = red alliance, false = blue alliance.
        const val DEFAULT_TO_RED_ALLIANCE = true

        const val MAX_SPEED = 1.0
        const val MAX_ANGULAR_SPEED = 1.0
        const val SLOW_DISTANCE = 1.0
        const val MIN_SPEED = 0.2
    }
}
