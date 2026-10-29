/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.aiming

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.AimingConstants
import frc.robot.Constants.FieldConstants
import frc.robot.Constants.ShotConstants
import frc.robot.subsystems.DeliverySubsystem
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.Telemetry
import frc.robot.util.Tunables
import kotlin.math.PI

/**
 * Feeds real robot data (pose, velocity, alliance) into [ShotMath] and publishes what it decides.
 *
 * Distance comes from the fused pose estimate, not straight from the camera, so it updates every
 * 20 ms even though the Limelight only runs at 15 fps.
 *
 * This is a SubsystemBase only so periodic() keeps "Shot/DistanceM" etc. on the dashboard even when
 * nobody is shooting (handy for tuning). No command ever requires it.
 */
object HubTargeting : SubsystemBase() {
    private const val TUNING_MODE_KEY = "Shot/TuningMode"
    private const val TUNING_RPS_KEY = "Shot/TuningRps"

    private val baseParams =
        ShotParams(
            flywheelRpsTable = LinearTable(ShotConstants.FLYWHEEL_RPS_BY_DISTANCE),
            timeOfFlightTable = LinearTable(ShotConstants.TIME_OF_FLIGHT_BY_DISTANCE),
            shooterOffsetRobot = Vec2(ShotConstants.SHOOTER_OFFSET_X, ShotConstants.SHOOTER_OFFSET_Y),
            shooterFacingRad = ShotConstants.SHOOTER_FACING_RAD,
            lookaheadSec = ShotConstants.LOOKAHEAD_SEC,
            timeOfFlightScale = ShotConstants.TOF_SCALE,
            allowedLateralErrorM = ShotConstants.ALLOWED_LATERAL_ERROR_M,
        )

    // The two moving-shot knobs are live-tunable (Tune/LookaheadSec, Tune/TofScale).
    private val params: ShotParams
        get() =
            baseParams.copy(
                lookaheadSec = Tunables.number("Tune/LookaheadSec", ShotConstants.LOOKAHEAD_SEC),
                timeOfFlightScale = Tunables.number("Tune/TofScale", ShotConstants.TOF_SCALE),
            )

    /** Shortest and longest distance in the flywheel table: outside it we have no data. */
    val tableMinM: Double get() = baseParams.flywheelRpsTable.minX
    val tableMaxM: Double get() = baseParams.flywheelRpsTable.maxX

    private val maxAlignAngleRad = Math.toRadians(ShotConstants.MAX_ALIGN_ANGLE_DEG)

    init {
        SmartDashboard.setDefaultBoolean(TUNING_MODE_KEY, false)
        SmartDashboard.setDefaultNumber(TUNING_RPS_KEY, ShotConstants.FLYWHEEL_RPS_BY_DISTANCE.first().second)

        // Put every switch and live gain on the dashboard at startup, so they can be set BEFORE the
        // first time B or X is pressed (e.g. turn on Shot/DryRun first). The commands read the same
        // keys with the same defaults.
        Tunables.flag("Shot/DryRun", false)
        Tunables.flag("Shot/IgnoreAllianceZone", false)
        Tunables.flag("Vision/Enabled", true)
        Tunables.number("Tune/AimP", ShotConstants.AIM_P)
        Tunables.number("Tune/AimD", ShotConstants.AIM_D)
        Tunables.number("Tune/MaxAimOmega", ShotConstants.MAX_AIM_OMEGA)
        Tunables.number("Tune/MaxDriveSpeedWhileShooting", ShotConstants.MAX_DRIVE_SPEED_WHILE_SHOOTING)
        Tunables.number("Tune/FlywheelToleranceRps", ShotConstants.FLYWHEEL_TOLERANCE_RPS)
        Tunables.number("Tune/LookaheadSec", ShotConstants.LOOKAHEAD_SEC)
        Tunables.number("Tune/TofScale", ShotConstants.TOF_SCALE)
        Tunables.number("Tune/AlignP", ShotConstants.ALIGN_P)
        Tunables.number("Tune/AlignMaxSpeed", ShotConstants.ALIGN_MAX_SPEED)
    }

    fun isRedAlliance(): Boolean =
        DriverStation
            .getAlliance()
            .map { it == DriverStation.Alliance.Red }
            .orElse(AimingConstants.DEFAULT_TO_RED_ALLIANCE)

    fun hubCenter(): Vec2 =
        if (isRedAlliance()) {
            Vec2(FieldConstants.RED_HUB_X, FieldConstants.HUB_Y)
        } else {
            Vec2(FieldConstants.BLUE_HUB_X, FieldConstants.HUB_Y)
        }

    /** Solve the shot for a robot at [pose] moving at [robotRelativeSpeeds] (as the drivetrain reports them). */
    fun solve(
        pose: Pose2d,
        robotRelativeSpeeds: ChassisSpeeds,
    ): ShotSolution {
        val fieldVelocity =
            Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
                .rotateBy(pose.rotation)
        return ShotMath.solve(
            robotPosition = Vec2(pose.x, pose.y),
            robotHeadingRad = pose.rotation.radians,
            robotFieldVelocity = Vec2(fieldVelocity.x, fieldVelocity.y),
            robotOmegaRadPerSec = robotRelativeSpeeds.omegaRadiansPerSecond,
            target = hubCenter(),
            params = params,
        )
    }

    /**
     * G407: we may only launch into the HUB while our bumpers are at least partly in our ALLIANCE ZONE.
     *
     * [allowPracticeOverride]: on a practice field the zone may not match, so "Shot/IgnoreAllianceZone"
     * can switch the check off. It is ignored whenever the robot is connected to a real match (FMS).
     */
    fun inAllianceZone(
        pose: Pose2d,
        allowPracticeOverride: Boolean = false,
    ): Boolean {
        val overridden =
            allowPracticeOverride &&
                Tunables.flag("Shot/IgnoreAllianceZone", false) &&
                !DriverStation.isFMSAttached()
        return overridden ||
            FieldRules.inAllianceZone(
                robotX = pose.x,
                isRed = isRedAlliance(),
                fieldLength = FieldConstants.FIELD_LENGTH,
                zoneDepth = FieldConstants.ALLIANCE_ZONE_DEPTH,
                robotHalfLength = FieldConstants.ROBOT_HALF_LENGTH,
                safetyMargin = ShotConstants.ZONE_SAFETY_MARGIN,
            )
    }

    /** Whether FUEL scored right now counts, from the FMS game data and match timer (manual 6.4.1). */
    fun isHubActive(): Boolean =
        FieldRules.isHubActive(
            isRed = DriverStation.getAlliance().map { it == DriverStation.Alliance.Red }.orElse(null),
            autoEnabled = DriverStation.isAutonomousEnabled(),
            teleopEnabled = DriverStation.isTeleopEnabled(),
            gameData = DriverStation.getGameSpecificMessage(),
            matchTimeSec = DriverStation.getMatchTime(),
        )

    /** Auto-align goal: SWEET_SPOT_DISTANCE from the HUB, in front of it, shooter pointed at it. */
    fun sweetSpotPose(pose: Pose2d): Pose2d {
        val hub = hubCenter()
        val spot =
            AlignMath.sweetSpot(
                robot = Vec2(pose.x, pose.y),
                hub = hub,
                distanceM = ShotConstants.SWEET_SPOT_DISTANCE,
                // Straight out from the HUB toward our ALLIANCE WALL (the blue wall is at x = 0).
                frontDirectionRad = if (isRedAlliance()) 0.0 else PI,
                maxAngleRad = maxAlignAngleRad,
            )
        // The shooter offset depends on heading, so solve twice: once to get close, once to settle.
        var heading = (hub - spot).angle - ShotConstants.SHOOTER_FACING_RAD
        repeat(2) {
            heading = ShotMath.solve(spot, heading, Vec2.ZERO, 0.0, hub, params).robotHeadingSetpointRad
        }
        return Pose2d(spot.x, spot.y, Rotation2d(heading))
    }

    /** Flywheel speed typed into the dashboard while "Shot/TuningMode" is on, otherwise null. */
    fun tuningRps(): Double? =
        if (SmartDashboard.getBoolean(TUNING_MODE_KEY, false)) SmartDashboard.getNumber(TUNING_RPS_KEY, 0.0) else null

    fun showAlignTarget(target: Pose2d?) {
        Telemetry.field.getObject("AlignTarget").setPoses(listOfNotNull(target))
    }

    /** [status] is "FIRING", or the reason the shot is blocked (see FeedGate). */
    fun publishShotState(
        headingErrorRad: Double,
        aimed: Boolean,
        atSpeed: Boolean,
        status: String,
    ) {
        SmartDashboard.putNumber("Shot/HeadingErrorDeg", Math.toDegrees(headingErrorRad))
        SmartDashboard.putBoolean("Shot/Aimed", aimed)
        SmartDashboard.putBoolean("Shot/AtSpeed", atSpeed)
        SmartDashboard.putBoolean("Shot/Firing", status == FIRING)
        SmartDashboard.putString("Shot/BlockedBy", status)
    }

    const val FIRING = "FIRING"

    override fun periodic() {
        val pose = DriveSubsystem.getPose()
        val shot = solve(pose, DriveSubsystem.getCurrentSpeeds())
        SmartDashboard.putNumber("Shot/DistanceM", shot.distanceM)
        SmartDashboard.putNumber("Shot/TableRps", shot.flywheelRps)
        SmartDashboard.putNumber("Shot/FlywheelRps", DeliverySubsystem.velocityRps())
        SmartDashboard.putNumber("Shot/TimeOfFlightSec", shot.timeOfFlightSec)
        SmartDashboard.putBoolean("Shot/InRange", shot.inRange)
        SmartDashboard.putBoolean("Shot/InAllianceZone", inAllianceZone(pose))
        SmartDashboard.putNumber("Pose/X", pose.x)
        SmartDashboard.putNumber("Pose/Y", pose.y)
        SmartDashboard.putNumber("Pose/HeadingDeg", pose.rotation.degrees)
        SmartDashboard.putBoolean("Shot/HubActive", isHubActive())
        Telemetry.field.getObject("ShotVirtualTarget").pose =
            Pose2d(shot.virtualTarget.x, shot.virtualTarget.y, Rotation2d.kZero)
    }
}
