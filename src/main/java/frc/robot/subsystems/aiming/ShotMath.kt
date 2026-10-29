/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.aiming

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

/*
 * Pure math for shooting while moving.
 *
 * Nothing in this file imports WPILib on purpose: the physics lives in one place and can be
 * unit tested without a robot. HubTargeting.kt is the thin layer that feeds it real robot data.
 *
 * Coordinates: WPILib blue-origin field frame. Meters, seconds, radians, CCW positive.
 */

private const val DEFAULT_MIN_AIM_TOLERANCE_DEG = 1.5

/** Small 2D vector (meters, or meters per second). */
data class Vec2(
    val x: Double,
    val y: Double,
) {
    operator fun plus(other: Vec2) = Vec2(x + other.x, y + other.y)

    operator fun minus(other: Vec2) = Vec2(x - other.x, y - other.y)

    operator fun times(scalar: Double) = Vec2(x * scalar, y * scalar)

    operator fun unaryMinus() = Vec2(-x, -y)

    val norm: Double get() = hypot(x, y)

    /** Direction of this vector, radians in (-PI, PI]. */
    val angle: Double get() = atan2(y, x)

    fun rotate(radians: Double): Vec2 {
        val c = cos(radians)
        val s = sin(radians)
        return Vec2(x * c - y * s, x * s + y * c)
    }

    /** Z component of the 3D cross product (this x other). */
    fun cross(other: Vec2): Double = x * other.y - y * other.x

    companion object {
        val ZERO = Vec2(0.0, 0.0)
    }
}

/** Piecewise-linear lookup table. Outside the measured range it holds the end values. */
class LinearTable(
    points: List<Pair<Double, Double>>,
) {
    private val xs: DoubleArray
    private val ys: DoubleArray

    init {
        require(points.size >= 2) { "LinearTable needs at least 2 points" }
        val sorted = points.sortedBy { it.first }
        xs = DoubleArray(sorted.size) { sorted[it].first }
        ys = DoubleArray(sorted.size) { sorted[it].second }
        for (i in 1 until xs.size) {
            require(xs[i] > xs[i - 1]) { "LinearTable x values must be unique" }
        }
    }

    val minX: Double get() = xs.first()
    val maxX: Double get() = xs.last()
    val maxY: Double get() = ys.max()

    operator fun get(x: Double): Double {
        if (x <= xs.first()) return ys.first()
        if (x >= xs.last()) return ys.last()
        var hi = 1
        while (xs[hi] < x) hi++
        val lo = hi - 1
        val t = (x - xs[lo]) / (xs[hi] - xs[lo])
        return ys[lo] + t * (ys[hi] - ys[lo])
    }
}

/** Everything about the shooter that the solver needs. All of it comes from Constants.ShooterConstants. */
data class ShotParams(
    /** Distance from the shooter to the HUB center (m) -> flywheel motor speed (rotations/s). */
    val flywheelRpsTable: LinearTable,
    /** Distance from the shooter to the HUB center (m) -> FUEL time of flight (s). */
    val timeOfFlightTable: LinearTable,
    /** Where FUEL leaves the robot, relative to robot center, in the robot frame (+x forward, +y left). */
    val shooterOffsetRobot: Vec2 = Vec2.ZERO,
    /** Direction FUEL leaves, relative to the robot's front. 0 = out the front, PI = out the back. */
    val shooterFacingRad: Double = 0.0,
    /** Predict the robot this far into the future (loop delay + time for FUEL to reach the flywheel). */
    val lookaheadSec: Double = 0.0,
    /** Multiplies every time-of-flight value. Shots land ahead of the robot's travel: raise it. Behind: lower it. */
    val timeOfFlightScale: Double = 1.0,
    /** Lateral miss (m) we accept at the HUB. Turned into an angle tolerance that shrinks with distance. */
    val allowedLateralErrorM: Double = 0.25,
    /** Never require tighter aim than this, so the robot does not hunt forever up close. */
    val minAimToleranceRad: Double = Math.toRadians(DEFAULT_MIN_AIM_TOLERANCE_DEG),
    /** Bisection steps for the time of flight. 30 steps resolves it to well under a microsecond. */
    val iterations: Int = 30,
)

data class ShotSolution(
    /** Field heading the ROBOT should face (already corrected for which way the shooter points). */
    val robotHeadingSetpointRad: Double,
    /** How fast that setpoint is moving because the robot is translating. Feed this forward. */
    val headingFeedforwardRadPerSec: Double,
    /** Distance from the shooter to the virtual target (what the flywheel table is indexed by). */
    val distanceM: Double,
    val flywheelRps: Double,
    val timeOfFlightSec: Double,
    /** The point we actually aim at: the HUB shifted backwards by robot velocity x time of flight. */
    val virtualTarget: Vec2,
    val shooterPosition: Vec2,
    /** True if the distance is inside the tuned table (we have real data there). */
    val inRange: Boolean,
    /** Heading error we can accept at this distance, radians. */
    val aimToleranceRad: Double,
)

object ShotMath {
    /**
     * Solve a shot at [target] from a robot that is moving.
     *
     * The FUEL keeps the robot's velocity after it leaves. Relative to the ground it lands at
     * (where it would land if we were still) + (robot velocity x time of flight). So instead of
     * aiming at the HUB we aim at the "virtual target" = HUB - velocity x time of flight, and pick
     * flywheel speed for the distance to that point. Time of flight depends on distance, so we
     * solve for the time of flight that is consistent with its own distance.
     *
     * Moving toward the HUB shortens the virtual distance (less flywheel), moving away lengthens it,
     * and moving sideways makes the robot lead the shot. That is how the robot's speed changes the
     * outtake speed: through the virtual distance, not as a separate fudge term.
     */
    @Suppress("LongParameterList")
    fun solve(
        robotPosition: Vec2,
        robotHeadingRad: Double,
        robotFieldVelocity: Vec2,
        robotOmegaRadPerSec: Double,
        target: Vec2,
        params: ShotParams,
    ): ShotSolution {
        // 1. Latency compensation: where will the robot be when the FUEL actually leaves?
        val predictedPosition = robotPosition + robotFieldVelocity * params.lookaheadSec
        val predictedHeading = robotHeadingRad + robotOmegaRadPerSec * params.lookaheadSec

        val now = aim(predictedPosition, predictedHeading, robotFieldVelocity, robotOmegaRadPerSec, target, params)

        // 2. Feedforward: how fast the heading setpoint moves while we translate. Solving the same
        //    shot a moment later captures both the robot moving and the virtual target sliding as
        //    the time of flight changes, which a hand-derived formula misses.
        val later =
            aim(
                predictedPosition + robotFieldVelocity * FEEDFORWARD_DT,
                predictedHeading,
                robotFieldVelocity,
                robotOmegaRadPerSec,
                target,
                params,
            )
        val feedforward = wrapAngle(later.heading - now.heading) / FEEDFORWARD_DT

        return ShotSolution(
            robotHeadingSetpointRad = now.heading,
            headingFeedforwardRadPerSec = if (now.distance > MIN_DISTANCE_FOR_FEEDFORWARD) feedforward else 0.0,
            distanceM = now.distance,
            flywheelRps = params.flywheelRpsTable[now.distance],
            timeOfFlightSec = now.timeOfFlight,
            virtualTarget = now.virtualTarget,
            shooterPosition = now.shooterPosition,
            inRange = now.distance >= params.flywheelRpsTable.minX && now.distance <= params.flywheelRpsTable.maxX,
            aimToleranceRad = aimTolerance(now.distance, params),
        )
    }

    private class Aim(
        val heading: Double,
        val distance: Double,
        val timeOfFlight: Double,
        val virtualTarget: Vec2,
        val shooterPosition: Vec2,
    )

    @Suppress("LongParameterList")
    private fun aim(
        robotPosition: Vec2,
        robotHeadingRad: Double,
        robotFieldVelocity: Vec2,
        robotOmegaRadPerSec: Double,
        target: Vec2,
        params: ShotParams,
    ): Aim {
        // Shooter exit point and its velocity (rigid body: v + omega x r).
        val offsetField = params.shooterOffsetRobot.rotate(robotHeadingRad)
        val shooterPosition = robotPosition + offsetField
        val shooterVelocity =
            robotFieldVelocity + Vec2(-robotOmegaRadPerSec * offsetField.y, robotOmegaRadPerSec * offsetField.x)

        // Virtual target. Find time of flight t with t = tofTable(|target - v*t - shooter|).
        // Plain fixed-point iteration converges slowly (and oscillates) when driving straight at
        // the HUB, so use bisection: mismatch(0) > 0 and mismatch(hi) < 0, so it always converges.
        fun mismatch(t: Double): Double =
            params.timeOfFlightTable[(target - shooterVelocity * t - shooterPosition).norm] *
                params.timeOfFlightScale - t
        var lo = 0.0
        var hi = params.timeOfFlightTable.maxY * params.timeOfFlightScale + 1.0
        repeat(params.iterations) {
            val mid = (lo + hi) / 2
            if (mismatch(mid) > 0.0) lo = mid else hi = mid
        }
        val timeOfFlight = (lo + hi) / 2
        val virtualTarget = target - shooterVelocity * timeOfFlight
        val toTarget = virtualTarget - shooterPosition

        // The robot faces the virtual target, rotated by however the shooter is mounted.
        return Aim(
            heading = wrapAngle(toTarget.angle - params.shooterFacingRad),
            distance = toTarget.norm,
            timeOfFlight = timeOfFlight,
            virtualTarget = virtualTarget,
            shooterPosition = shooterPosition,
        )
    }

    /** Angle tolerance that keeps the lateral miss under params.allowedLateralErrorM. */
    fun aimTolerance(
        distanceM: Double,
        params: ShotParams,
    ): Double = max(params.minAimToleranceRad, atan2(params.allowedLateralErrorM, distanceM))

    /** Wrap an angle to [-PI, PI). */
    fun wrapAngle(radians: Double): Double {
        var a = (radians + PI) % (2 * PI)
        if (a < 0) a += 2 * PI
        return a - PI
    }

    private const val MIN_DISTANCE_FOR_FEEDFORWARD = 0.05
    private const val FEEDFORWARD_DT = 0.01
}

/** Decides whether the indexer may feed, and if not, says why in words a driver understands. */
object FeedGate {
    data class Inputs(
        val distanceM: Double,
        val inRange: Boolean,
        val tableMinM: Double,
        val tableMaxM: Double,
        val inAllianceZone: Boolean,
        val headingErrorRad: Double,
        val aimToleranceRad: Double,
        val flywheelRps: Double,
        val targetRps: Double,
        val flywheelToleranceRps: Double,
        val dryRun: Boolean,
    )

    /** The first thing stopping the shot, or null when it is good to fire. Checked in the order a driver fixes them. */
    fun blockedBy(i: Inputs): String? =
        when {
            !i.inRange ->
                "out of range: %.1f m (table covers %.1f-%.1f m)".format(i.distanceM, i.tableMinM, i.tableMaxM)
            !i.inAllianceZone -> "outside alliance zone (G407)"
            abs(i.headingErrorRad) > i.aimToleranceRad ->
                "aiming: %.1f deg off, need under %.1f".format(
                    Math.toDegrees(abs(i.headingErrorRad)),
                    Math.toDegrees(i.aimToleranceRad),
                )
            abs(i.flywheelRps - i.targetRps) > i.flywheelToleranceRps ->
                "flywheel at %.0f of %.0f rps".format(i.flywheelRps, i.targetRps)
            i.dryRun -> "dry run: shot is ready, indexer held off"
            else -> null
        }
}

/** 2026 REBUILT rules that decide whether a shot is worth taking. */
object FieldRules {
    /**
     * G407: a robot may only launch FUEL into its HUB while its BUMPERS are at least partly inside
     * its ALLIANCE ZONE (MAJOR FOUL otherwise). The rule is about bumpers, so we allow the robot
     * center to be up to half a robot length past the line, minus a safety margin for pose error.
     */
    @Suppress("LongParameterList")
    fun inAllianceZone(
        robotX: Double,
        isRed: Boolean,
        fieldLength: Double,
        zoneDepth: Double,
        robotHalfLength: Double,
        safetyMargin: Double,
    ): Boolean {
        val reach = zoneDepth + robotHalfLength - safetyMargin
        return if (isRed) robotX >= fieldLength - reach else robotX <= reach
    }

    /**
     * Port of WPILib's "2026 Game Data Details" example. Game data is one character, the alliance
     * whose HUB goes inactive first ('R' or 'B'), sent about 3 s after AUTO. That alliance is
     * inactive in SHIFT 1 and 3 and active in SHIFT 2 and 4. Match time is approximate.
     */
    @Suppress("MagicNumber", "ReturnCount")
    fun isHubActive(
        isRed: Boolean?,
        autoEnabled: Boolean,
        teleopEnabled: Boolean,
        gameData: String,
        matchTimeSec: Double,
    ): Boolean {
        if (isRed == null) return false
        if (autoEnabled) return true
        if (!teleopEnabled) return false
        if (gameData.isEmpty()) return true // not received yet
        val first = gameData[0]
        if (first != 'R' && first != 'B') return true // corrupt data, do not block shooting
        val redInactiveFirst = first == 'R'
        val shift1Active = if (isRed) !redInactiveFirst else redInactiveFirst
        return when {
            matchTimeSec > 130 -> true // TRANSITION SHIFT
            matchTimeSec > 105 -> shift1Active // SHIFT 1
            matchTimeSec > 80 -> !shift1Active // SHIFT 2
            matchTimeSec > 55 -> shift1Active // SHIFT 3
            matchTimeSec > 30 -> !shift1Active // SHIFT 4
            else -> true // END GAME
        }
    }
}

/** Where auto-align drives to. */
object AlignMath {
    private const val MIN_BEARING_DISTANCE = 1e-6

    /**
     * The point [distanceM] from [hub] in the direction of [robot], with that direction kept within
     * [maxAngleRad] of [frontDirectionRad] (straight out from the HUB toward our ALLIANCE WALL).
     * Most of the time this makes auto-align a straight move toward or away from the HUB.
     */
    fun sweetSpot(
        robot: Vec2,
        hub: Vec2,
        distanceM: Double,
        frontDirectionRad: Double,
        maxAngleRad: Double,
    ): Vec2 {
        val toRobot = robot - hub
        val bearing = if (toRobot.norm < MIN_BEARING_DISTANCE) frontDirectionRad else toRobot.angle
        val offset = ShotMath.wrapAngle(bearing - frontDirectionRad).coerceIn(-maxAngleRad, maxAngleRad)
        val direction = frontDirectionRad + offset
        return hub + Vec2(cos(direction), sin(direction)) * distanceM
    }
}
