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

import com.ctre.phoenix6.CANBus
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI

object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val DRIVE_DEADBAND = 0.2
        const val LERP_VAL = 0.035

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 9 // joystick
        const val ALIGN_RIGHT_BUTTON = 10 // joystick

        const val BUTTON_BOARD_PORT = 2

        const val ANGLE_BUTTON = 12 // joystick

        const val INTAKE_BUTTON = 10
        const val INTAKE_REVERSE_BUTTON = 2
        const val INTAKE_LEVER_OUT_MANUAL_BUTTON = 5
        const val INTAKE_LEVER_IN_MANUAL_BUTTON = 4
        const val INTAKE_LEVER_OUT_AUTO_BUTTON = 8
        const val INTAKE_LEVER_IN_AUTO_BUTTON = 9
        const val DELIVERY_BUTTON = 12
        const val DELIVERY_REVERSE_BUTTON = 1
        const val INDEXER_REVERSE_BUTTON = 3
    }

    object IntakeConstants {
        const val INTAKE_MOTOR_ID = 34
        const val RIGHT_LEVER_MOTOR_ID = 31
        const val LEFT_LEVER_MOTOR_ID = 30
        const val INTAKE_SPEED = 0.8

        // Limits should be in rotations
        const val LEVER_LIMIT_FORWARD = 0.0
        const val LEVER_LIMIT_REVERSE = 0.25

        const val P = 2.0
        const val I = 0.0
        const val D = 0.1
        const val kG = 0.008

        // basically a controller deadzone, how close to the setpoint it needs to be to stop moving
        const val GAIN_SCHEDULE_ERROR_THRESHOLD = 0.01

        // horizontal needs to be 0
        // 12 rotations motor = out to in

        // now in mechanism rotations

        const val LEVER_OUT_POSITION = 0.0
        const val LEVER_IN_POSITION = 0.25

        const val LEVER_SPEED = 0.2 // in rpm

        const val INTAKE_CURRENT_LIMIT = 20.0
        const val INTAKE_STATOR_LIMIT = 40.0
    }

    enum class IntakeDirection {
        IN,
        OUT,
    }

    object SomeConstants {
        val isReal: Boolean = RobotBase.isReal()

        enum class SwerveType {
            TALON,
            SPARKMAX,
        }
    }

    object TelemetryConstants {
        const val MOTOR_CURRENT_MIN_RANGE = 0.50 //amps
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 5.0
        const val MAX_ANGULAR_SPEED = 5.0
        const val MAX_SLIDING_SPEED_PERCENTAGE = 0.5

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

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.831299)
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.113525 + 0.5)
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(-0.170166) // + is clockwise
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.002686 + 0.5)
        // - counter-clockwise
        //   back right - > front left
        //   back left - >front right
        //   front left -> back right
        // front right -> back left

        const val FRONT_LEFT_DRIVING_ID = 8 // 8->4
        const val REAR_LEFT_DRIVING_ID = 7 // 5->19
        const val FRONT_RIGHT_DRIVING_ID = 19 // 19->5
        const val REAR_RIGHT_DRIVING_ID = 4 // 4->8

        const val FRONT_LEFT_TURNING_ID = 2 // 2->3
        const val REAR_LEFT_TURNING_ID = 5 // 7->6
        const val FRONT_RIGHT_TURNING_ID = 6 // 6->7
        const val REAR_RIGHT_TURNING_ID = 3 // 3->2

        const val FRONT_LEFT_CANCODER_ID = 11 // 11->9
        const val REAR_LEFT_CANCODER_ID = 12 // 12->10
        const val FRONT_RIGHT_CANCODER_ID = 10 // 10->12
        const val REAR_RIGHT_CANCODER_ID = 9 // 9->11

        const val ROTATE_CONTROLLER_P = 2.0
        const val ROTATE_CONTROLLER_I = 0.0
        const val ROTATE_CONTROLLER_D = 0.01
        const val TRANSLATION_CONTROLLER_P = 5.0
        const val TRANSLATION_CONTROLLER_I = 0.0
        const val TRANSLATION_CONTROLLER_D = 0.01

        const val DRIVE_SETPOINT_TOLERANCE = 0.05
        const val ROTATE_SETPOINT_TOLERANCE = 0.08

        const val PIDGEON2_ID = 20
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

        const val DRIVING_MOTOR_CURRENT_LIMIT = 60.0
        const val TURNING_MOTOR_CURRENT_LIMIT = 60.0
        const val DRIVING_STATOR_CURRENT_LIMIT = 120.0
        const val TURNING_STATOR_CURRENT_LIMIT = 120.0

        val CANBUS: CANBus = CANBus("didy")
    }

    object LimelightConstants {
        const val POLLING_RATE = 20L
        const val TIMEOUT = 500L // milliseconds
        const val IP_ADDR = "10.66.95.201"
//        const val IP_ADDR = "172.29.0.1"
    }

    object LaunchConstants {
        const val LEFT_LAUNCHMOTOR_ID = 54
        const val RIGHT_LAUNCHMOTOR_ID = 28
        const val LAUNCH_SPEED = 0.65
        const val ALT_LAUNCH_SPEED = 0.85

        // Slot0 gains, only used by closed-loop velocity (DeliverySubsystem.setVelocity).
        // The duty-cycle buttons (forward()) ignore these. Units are Phoenix 6: volts per rotation/s.
        const val LAUNCH_P = 0.2 // TUNE: raise until the flywheel recovers fast after each FUEL
        const val LAUNCH_I = 0.0
        const val LAUNCH_D = 0.0
        const val LAUNCH_FF = 0.15 // kS: volts to overcome friction
        const val LAUNCH_V = 0.12 // kV: ~12 V / 100 rps free speed for a Kraken X60. TUNE
        const val LAUNCH_A = 0.0
        const val LAUNCH_MOTOR_CURRENT_LIMITS = 40.0
        const val LAUNCH_MOTOR_STATOR_LIMITS = 80.0
    }

    object RollerConstants {
        const val ROLLER_MOTOR_ID = 23

        const val ROLLER_MOTOR_ID_2 = 24
        const val ROLLER_SPEED = 0.65
        const val ALT_ROLLER_SPEED = 0.9
        const val ROLLER_MOTOR_CURRENT_LIMITS = 20.0
        const val ROLLER_MOTOR_STATOR_LIMITS = 40.0
        const val BUTTON = 11
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

        // HUB centers in the WPILib blue-origin field frame (same frame as PathPlanner and the
        // pose estimator). Values come from FieldConstants, which reads them off the official layout.
        const val RED_HUB_X = FieldConstants.RED_HUB_X
        const val RED_HUB_Y = FieldConstants.HUB_Y
        const val BLUE_HUB_X = FieldConstants.BLUE_HUB_X
        const val BLUE_HUB_Y = FieldConstants.HUB_Y

        // Valid shooting-arc sector, in degrees, measured from hub center.
        //   0° = +X on field,  90° = +Y,  180° = -X,  270° = -Y (toward driver station)
        // These values apply for red alliance (robot below hub, y < RED_HUB_Y).
        // For blue alliance the sector is mirrored vertically — team must tune.
        const val MIN_ANGLE_DEGREES = 0.0
        const val MAX_ANGLE_DEGREES = 180.0

        // Fallback when DriverStation hasn't reported an alliance yet (e.g. practice mode).
        // true = red alliance, false = blue alliance.
        const val DEFAULT_TO_RED_ALLIANCE = true

        const val MAX_SPEED = 1.0
        const val MAX_ANGULAR_SPEED = 1.0
        const val SLOW_DISTANCE = 1.0
        const val MIN_SPEED = 0.2

        const val DIST_DEADZONE = 0.1 // m
        const val ANGLE_DEADZONE = 1.0 // degrees
    }

    /** 2026 REBUILT field in the WPILib blue-origin frame (meters), from 2026-rebuilt-welded.json. */
    object FieldConstants {
        const val FIELD_LENGTH = 16.541
        const val FIELD_WIDTH = 8.069

        // HUB center = midpoint of the centered AprilTags on opposite HUB faces
        // (blue: tags 18/21 and 20/26, red: tags 2/5 and 4/10).
        const val BLUE_HUB_X = 4.6256
        const val RED_HUB_X = 11.9154
        const val HUB_Y = 4.0346

        // Game manual 5.3: the ALLIANCE ZONE is 158.6 in deep, measured from the ALLIANCE WALL.
        const val ALLIANCE_ZONE_DEPTH = 4.028

        // Half the robot length with bumpers (PathPlanner settings: 0.914 m square).
        const val ROBOT_HALF_LENGTH = 0.457
    }

    object VisionConstants {
        // Must match the Limelight's hostname in its web UI (default "limelight").
        const val LIMELIGHT_NAME = "limelight"

        // Frames the Limelight skips between processed frames. Limelight recommends 100-200 while
        // disabled to keep it cool. 0 = process every frame while enabled.
        const val THROTTLE_DISABLED = 100
        const val THROTTLE_ENABLED = 0

        // Ignore MegaTag2 while spinning fast or when the tags are far away.
        const val MAX_YAW_RATE_DEG_PER_SEC = 360.0
        const val MAX_TAG_DISTANCE_M = 6.0

        // How much to trust vision: XY std dev = XY_STD_DEV_PER_M2 * avgTagDist^2 / tagCount, never below MIN.
        const val XY_STD_DEV_PER_M2 = 0.1
        const val MIN_XY_STD_DEV = 0.05

        // While disabled, a MegaTag1 solve with at least this many tags resets the whole pose,
        // including heading. MegaTag2 needs a correct field heading to work.
        const val SEED_MIN_TAGS = 2
        const val SEED_MAX_TAG_DISTANCE_M = 4.0
    }

    object ShotConstants {
        // ---- Lookup tables: THE thing to tune. Distance = shooter exit point to HUB center (m). ----
        // Flywheel motor speed (rotations/s) that scores from a STANDING robot at that distance.
        // Placeholders! Fill these in using "Shot/TuningMode" (see ShootOnTheMoveCommand).
        @Suppress("MagicNumber")
        val FLYWHEEL_RPS_BY_DISTANCE =
            listOf(
                1.0 to 58.0,
                2.0 to 64.0,
                3.0 to 71.0,
                4.0 to 78.0,
                5.0 to 85.0,
                6.0 to 90.0,
            )

        // FUEL time of flight (s) from leaving the shooter to dropping into the HUB, at that distance.
        // Only matters while moving (it sets how far to lead). Measure with slow-motion video.
        @Suppress("MagicNumber")
        val TIME_OF_FLIGHT_BY_DISTANCE =
            listOf(
                1.0 to 0.55,
                2.0 to 0.90,
                3.0 to 1.15,
                4.0 to 1.35,
                5.0 to 1.55,
                6.0 to 1.70,
            )

        // Where FUEL leaves the robot, measured from robot center (m): +x forward, +y left.
        const val SHOOTER_OFFSET_X = 0.0 // MEASURE
        const val SHOOTER_OFFSET_Y = 0.0 // MEASURE

        // Which way the shooter points relative to the robot front: 0 = out the front, PI = out the back.
        // The autos start at heading 0 facing the HUB, so the shooter is assumed to face forward.
        const val SHOOTER_FACING_RAD = 0.0

        // Predict the robot this far ahead: loop delay + time for FUEL to travel from indexer to exit.
        const val LOOKAHEAD_SEC = 0.10 // TUNE

        // Multiplies the time-of-flight table. Moving shots land ahead of your travel: raise it. Behind: lower it.
        const val TOF_SCALE = 1.0

        // How far sideways from HUB center a shot can land and still go in (the opening is 1.06 m wide).
        const val ALLOWED_LATERAL_ERROR_M = 0.25

        const val FLYWHEEL_TOLERANCE_RPS = 2.0
        const val FEED_SPEED = 0.65 // indexer duty cycle while firing
        const val FEED_DEBOUNCE_SEC = 0.06 // must be ready this long before the indexer starts

        // The driver keeps translation while shooting, capped for accuracy.
        const val MAX_DRIVE_SPEED_WHILE_SHOOTING = 2.0 // m/s

        // Heading controller: rad/s per rad of error, added to the solver's feedforward.
        const val AIM_P = 5.0
        const val AIM_D = 0.1
        const val MAX_AIM_OMEGA = 5.0 // rad/s

        // ---- Auto-align: drive to a spot SWEET_SPOT_DISTANCE from HUB center, facing it. ----
        const val SWEET_SPOT_DISTANCE = 2.0 // m. TUNE: set to your most reliable shot distance

        // Stay within this many degrees of straight in front of the HUB, so the bumpers clear the BUMPs.
        const val MAX_ALIGN_ANGLE_DEG = 50.0
        const val ALIGN_P = 3.0 // m/s per m of error
        const val ALIGN_MAX_SPEED = 2.5 // m/s
        const val ALIGN_POSITION_TOLERANCE = 0.05 // m
        const val ALIGN_HEADING_TOLERANCE_DEG = 2.0

        // G407 margin for pose error when checking that the bumpers are in the ALLIANCE ZONE.
        const val ZONE_SAFETY_MARGIN = 0.15 // m

        // Time limits for the PathPlanner named commands "AlignToHub" and "AimAndShoot".
        const val AUTO_ALIGN_TIMEOUT_SEC = 3.0
        const val AUTO_SHOOT_TIMEOUT_SEC = 3.0
    }

    /** Desktop simulation only (`./gradlew simulateJava`). None of this affects the real robot. */
    object SimConstants {
        const val LOOP_SEC = 0.02

        // The sim robot starts this far from the blue wall, in line with the HUB and facing it (mirrored for red).
        const val START_X_BLUE = 2.5

        const val MAX_ACCEL = 8.0 // m/s^2
        const val MAX_ANGULAR_ACCEL = 30.0 // rad/s^2

        // Flywheel inertia (kg m^2). A guess: it only changes how fast the sim flywheel spins up.
        const val FLYWHEEL_MOI = 0.004
    }

    object PathPlannerConstants {
        const val TRANSLATION_P = 5.0
        const val TRANSLATION_I = 0.0
        const val TRANSLATION_D = 0.0

        const val ROTATION_P = 17.0
        const val ROTATION_I = 0.0
        const val ROTATION_D = 0.0

        val FIELD_SIZE = Translation2d(16.54099, 8.069326) // get from /deploy/pathplanner/navgrid.json
    }
}
