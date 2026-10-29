/*
 * (C) 2025 Galvaknights
 */
package frc.robot.aiming

import frc.robot.subsystems.aiming.AlignMath
import frc.robot.subsystems.aiming.FeedGate
import frc.robot.subsystems.aiming.FieldRules
import frc.robot.subsystems.aiming.LinearTable
import frc.robot.subsystems.aiming.ShotMath
import frc.robot.subsystems.aiming.ShotParams
import frc.robot.subsystems.aiming.Vec2
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

internal class ShotMathTest {
    // Blue HUB center from the 2026 welded AprilTag layout.
    private val hub = Vec2(4.6256, 4.0346)

    private val params =
        ShotParams(
            flywheelRpsTable =
                LinearTable(
                    listOf(1.5 to 59.0, 2.0 to 60.5, 3.0 to 68.0, 4.0 to 76.0, 5.0 to 84.0, 6.0 to 91.0),
                ),
            timeOfFlightTable =
                LinearTable(
                    listOf(1.5 to 0.50, 2.0 to 0.66, 3.0 to 0.89, 4.0 to 1.08, 5.0 to 1.25, 6.0 to 1.39),
                ),
        )

    private fun solve(
        position: Vec2,
        velocity: Vec2 = Vec2.ZERO,
        heading: Double = 0.0,
        omega: Double = 0.0,
        p: ShotParams = params,
    ) = ShotMath.solve(position, heading, velocity, omega, hub, p)

    @Test
    fun `table interpolates and clamps`() {
        val table = LinearTable(listOf(2.0 to 20.0, 1.0 to 10.0, 3.0 to 40.0))
        assertEquals(10.0, table[0.0], 1e-9)
        assertEquals(15.0, table[1.5], 1e-9)
        assertEquals(30.0, table[2.5], 1e-9)
        assertEquals(40.0, table[99.0], 1e-9)
        assertEquals(1.0, table.minX, 1e-9)
        assertEquals(3.0, table.maxX, 1e-9)
    }

    @Test
    fun `standing still aims straight at the hub`() {
        val robot = Vec2(hub.x - 3.0, hub.y)
        val s = solve(robot)
        assertEquals(0.0, s.robotHeadingSetpointRad, 1e-9)
        assertEquals(3.0, s.distanceM, 1e-9)
        assertEquals(68.0, s.flywheelRps, 1e-9)
        assertEquals(0.89, s.timeOfFlightSec, 1e-9)
        assertEquals(hub, s.virtualTarget)
        assertEquals(0.0, s.headingFeedforwardRadPerSec, 1e-9)
        assertTrue(s.inRange)
    }

    @Test
    fun `heading points from robot to hub in any direction`() {
        val below = solve(Vec2(hub.x, hub.y - 3.0))
        assertEquals(PI / 2, below.robotHeadingSetpointRad, 1e-9)
        val diagonal = solve(Vec2(hub.x - 2.0, hub.y + 2.0))
        assertEquals(-PI / 4, diagonal.robotHeadingSetpointRad, 1e-9)
    }

    @Test
    fun `fuel launched from a moving robot lands in the hub`() {
        // Strafing past the hub, driving at it, driving away, and diagonals.
        val velocities =
            listOf(
                Vec2(0.0, 2.0),
                Vec2(2.5, 0.0),
                Vec2(4.0, 0.0), // fast, straight at the hub: hardest case for the solver
                Vec2(-2.0, 0.0),
                Vec2(-3.5, 1.0),
                Vec2(1.5, -1.5),
                Vec2(-1.0, 2.5),
            )
        for (v in velocities) {
            val robot = Vec2(hub.x - 3.2, hub.y - 0.7)
            val s = solve(robot, v)
            // Ball model the tables describe: relative to the robot, FUEL covers the table
            // distance in the table time of flight. Add the robot's velocity it inherited.
            val aimDirection = (s.virtualTarget - s.shooterPosition) * (1.0 / s.distanceM)
            val tof = params.timeOfFlightTable[s.distanceM]
            val landing = s.shooterPosition + v * tof + aimDirection * s.distanceM
            assertEquals(0.0, (landing - hub).norm, 0.01, "missed hub with velocity $v")
        }
    }

    @Test
    fun `driving toward the hub needs less flywheel, away needs more`() {
        val robot = Vec2(hub.x - 3.0, hub.y)
        val still = solve(robot).flywheelRps
        val toward = solve(robot, Vec2(2.0, 0.0)).flywheelRps
        val away = solve(robot, Vec2(-2.0, 0.0)).flywheelRps
        assertTrue(toward < still, "toward=$toward still=$still")
        assertTrue(away > still, "away=$away still=$still")
    }

    @Test
    fun `strafing leads the shot and feeds rotation forward`() {
        // Robot west of the hub, strafing north (+y). The hub drifts to the robot's right,
        // so aim must lead to the right (negative angle) and the setpoint turns clockwise.
        val robot = Vec2(hub.x - 3.0, hub.y)
        val v = Vec2(0.0, 2.0)
        val s = solve(robot, v)
        assertTrue(s.robotHeadingSetpointRad < 0.0)
        assertTrue(s.virtualTarget.y < hub.y)
        // Feedforward must match how fast the setpoint actually moves as the robot drives.
        val dt = 1e-4
        val later = solve(robot + v * dt, v)
        val numeric = (later.robotHeadingSetpointRad - s.robotHeadingSetpointRad) / dt
        assertTrue(s.headingFeedforwardRadPerSec < 0.0)
        assertEquals(numeric, s.headingFeedforwardRadPerSec, 0.02)
    }

    @Test
    fun `feedforward matches setpoint motion for any velocity`() {
        val robot = Vec2(hub.x - 2.5, hub.y + 1.2)
        for (v in listOf(Vec2(1.0, 1.0), Vec2(-2.0, 0.5), Vec2(0.3, -2.4))) {
            val s = solve(robot, v)
            val dt = 1e-4
            val numeric = (solve(robot + v * dt, v).robotHeadingSetpointRad - s.robotHeadingSetpointRad) / dt
            assertEquals(numeric, s.headingFeedforwardRadPerSec, 0.05, "v=$v")
        }
    }

    @Test
    fun `shooter out the back flips the robot heading`() {
        val back = params.copy(shooterFacingRad = PI)
        val s = solve(Vec2(hub.x - 3.0, hub.y), p = back)
        assertEquals(PI, abs(s.robotHeadingSetpointRad), 1e-9)
    }

    @Test
    fun `shooter offset aims from the exit point not robot center`() {
        // Shooter 0.3 m to the robot's left. Robot faces +x, west of the hub.
        val offset = params.copy(shooterOffsetRobot = Vec2(0.0, 0.3))
        val s = solve(Vec2(hub.x - 3.0, hub.y), p = offset)
        assertEquals(Vec2(hub.x - 3.0, hub.y + 0.3), s.shooterPosition)
        assertEquals(atan2(-0.3, 3.0), s.robotHeadingSetpointRad, 1e-9)
    }

    @Test
    fun `lookahead predicts where the robot will be`() {
        val ahead = params.copy(lookaheadSec = 0.1)
        val s = solve(Vec2(hub.x - 3.0, hub.y), Vec2(1.0, 0.0), p = ahead)
        assertEquals(hub.x - 2.9, s.shooterPosition.x, 1e-9)
    }

    @Test
    fun `out of table range is flagged`() {
        assertFalse(solve(Vec2(hub.x - 7.0, hub.y)).inRange)
        assertFalse(solve(Vec2(hub.x - 1.0, hub.y)).inRange)
    }

    @Test
    fun `aim tolerance shrinks with distance but has a floor`() {
        val near = ShotMath.aimTolerance(2.0, params)
        val far = ShotMath.aimTolerance(5.0, params)
        assertTrue(near > far)
        assertEquals(atan2(0.25, 5.0), far, 1e-9)
        assertEquals(params.minAimToleranceRad, ShotMath.aimTolerance(100.0, params), 1e-9)
    }

    @Test
    fun `wrapAngle stays in range`() {
        assertEquals(0.0, ShotMath.wrapAngle(2 * PI), 1e-9)
        assertEquals(-PI / 2, ShotMath.wrapAngle(3 * PI / 2), 1e-9)
        assertEquals(PI / 2, ShotMath.wrapAngle(-3 * PI / 2), 1e-9)
        assertEquals(0.1, ShotMath.wrapAngle(0.1 + 4 * PI), 1e-9)
    }

    @Test
    fun `alliance zone follows G407 bumper rule`() {
        val fieldLength = 16.541

        fun blue(x: Double) = FieldRules.inAllianceZone(x, false, fieldLength, 4.028, 0.457, 0.15)

        fun red(x: Double) = FieldRules.inAllianceZone(x, true, fieldLength, 4.028, 0.457, 0.15)
        assertTrue(blue(2.0))
        assertTrue(blue(4.3)) // center past the line, bumpers still in
        assertFalse(blue(4.5))
        assertFalse(blue(12.0))
        assertTrue(red(fieldLength - 2.0))
        assertFalse(red(fieldLength - 4.5))
        assertFalse(red(3.0))
    }

    @Test
    fun `sweet spot is straight out from the hub when the robot is in front`() {
        val spot = AlignMath.sweetSpot(Vec2(hub.x - 3.0, hub.y), hub, 2.0, PI, Math.toRadians(50.0))
        assertEquals(hub.x - 2.0, spot.x, 1e-9)
        assertEquals(hub.y, spot.y, 1e-9)
    }

    @Test
    fun `sweet spot keeps the robot's bearing when it is inside the allowed angle`() {
        val robot = hub + Vec2(-3.0, -1.0) // ~18 degrees off straight in front
        val spot = AlignMath.sweetSpot(robot, hub, 2.0, PI, Math.toRadians(50.0))
        assertEquals(2.0, (spot - hub).norm, 1e-9)
        assertEquals((robot - hub).angle, (spot - hub).angle, 1e-9)
    }

    @Test
    fun `sweet spot clamps to the allowed angle when the robot is beside the hub`() {
        val maxAngle = Math.toRadians(50.0)
        val beside = AlignMath.sweetSpot(hub + Vec2(0.0, 3.0), hub, 2.0, PI, maxAngle)
        assertEquals(PI - maxAngle, (beside - hub).angle, 1e-9)
        val otherSide = AlignMath.sweetSpot(hub + Vec2(0.0, -3.0), hub, 2.0, PI, maxAngle)
        assertEquals(-(PI - maxAngle), (otherSide - hub).angle, 1e-9)
    }

    @Test
    fun `sweet spot works for red and for a robot sitting on the hub center`() {
        val redHub = Vec2(11.9154, 4.0346)
        val red = AlignMath.sweetSpot(redHub + Vec2(3.0, 0.0), redHub, 2.0, 0.0, Math.toRadians(50.0))
        assertEquals(redHub.x + 2.0, red.x, 1e-9)
        val degenerate = AlignMath.sweetSpot(hub, hub, 2.0, PI, Math.toRadians(50.0))
        assertEquals(hub.x - 2.0, degenerate.x, 1e-9)
    }

    @Test
    fun `every sweet spot with the default constants is a legal place to shoot`() {
        // SWEET_SPOT_DISTANCE = 2.0 and MAX_ALIGN_ANGLE_DEG = 50 in Constants.
        val fieldLength = 16.541
        for (deg in -180..180 step 5) {
            val a = Math.toRadians(deg.toDouble())
            val blue = AlignMath.sweetSpot(hub + Vec2(cos(a), sin(a)), hub, 2.0, PI, Math.toRadians(50.0))
            assertTrue(FieldRules.inAllianceZone(blue.x, false, fieldLength, 4.028, 0.457, 0.15), "blue $deg")
            val redHub = Vec2(fieldLength - hub.x, hub.y)
            val red = AlignMath.sweetSpot(redHub + Vec2(cos(a), sin(a)), redHub, 2.0, 0.0, Math.toRadians(50.0))
            assertTrue(FieldRules.inAllianceZone(red.x, true, fieldLength, 4.028, 0.457, 0.15), "red $deg")
        }
    }

    @Test
    fun `time of flight scale stretches the lead while moving`() {
        val robot = Vec2(hub.x - 3.0, hub.y)
        val v = Vec2(0.0, 2.0)
        val normal = solve(robot, v)
        val longer = solve(robot, v, p = params.copy(timeOfFlightScale = 1.2))
        // The solved flight time must satisfy its own definition: 1.2 x table(distance to the virtual target).
        assertEquals(params.timeOfFlightTable[longer.distanceM] * 1.2, longer.timeOfFlightSec, 1e-6)
        assertTrue(longer.timeOfFlightSec > normal.timeOfFlightSec * 1.2, "lead grows the distance, so even more")
        assertTrue(longer.virtualTarget.y < normal.virtualTarget.y, "more lead with a longer flight")
        // Standing still, the scale changes nothing about where we aim.
        val still = solve(robot, p = params.copy(timeOfFlightScale = 1.2))
        assertEquals(solve(robot).robotHeadingSetpointRad, still.robotHeadingSetpointRad, 1e-9)
    }

    @Test
    fun `too short a time of flight makes moving shots land ahead of the robot's travel`() {
        // This pins the tuning rule in the guide: lands ahead -> raise TofScale, lands behind -> lower it.
        val robot = Vec2(hub.x - 2.5, hub.y)
        val v = Vec2(0.0, 1.5) // strafing toward +y
        for ((scale, ahead) in listOf(0.8 to true, 1.25 to false)) {
            val s = solve(robot, v, p = params.copy(timeOfFlightScale = scale))
            // Real ball: flies the TRUE table time for the distance it was launched at.
            val landing = s.virtualTarget + v * params.timeOfFlightTable[s.distanceM]
            assertEquals(ahead, landing.y > hub.y, "scale $scale")
        }
    }

    private val ready =
        FeedGate.Inputs(
            distanceM = 2.5,
            inRange = true,
            tableMinM = 1.0,
            tableMaxM = 6.0,
            inAllianceZone = true,
            headingErrorRad = Math.toRadians(0.5),
            aimToleranceRad = Math.toRadians(2.0),
            flywheelRps = 64.0,
            targetRps = 65.0,
            flywheelToleranceRps = 2.0,
            dryRun = false,
        )

    @Test
    fun `feed gate fires only when everything is good`() {
        assertEquals(null, FeedGate.blockedBy(ready))
    }

    @Test
    fun `feed gate names the first problem in fix order`() {
        assertEquals(
            "out of range: 7.2 m (table covers 1.0-6.0 m)",
            FeedGate.blockedBy(ready.copy(inRange = false, distanceM = 7.2, inAllianceZone = false)),
        )
        assertEquals("outside alliance zone (G407)", FeedGate.blockedBy(ready.copy(inAllianceZone = false)))
        assertEquals(
            "aiming: 4.0 deg off, need under 2.0",
            FeedGate.blockedBy(ready.copy(headingErrorRad = Math.toRadians(-4.0))),
        )
        assertEquals("flywheel at 50 of 65 rps", FeedGate.blockedBy(ready.copy(flywheelRps = 50.0)))
        assertEquals("dry run: shot is ready, indexer held off", FeedGate.blockedBy(ready.copy(dryRun = true)))
    }

    @Test
    fun `hub active follows the shift table`() {
        // 'R' = red HUB goes inactive first.
        fun red(t: Double) = FieldRules.isHubActive(true, false, true, "R", t)

        fun blue(t: Double) = FieldRules.isHubActive(false, false, true, "R", t)
        assertTrue(red(135.0)) // transition
        assertFalse(red(120.0)) // shift 1
        assertTrue(red(95.0)) // shift 2
        assertFalse(red(70.0)) // shift 3
        assertTrue(red(40.0)) // shift 4
        assertTrue(red(10.0)) // end game
        assertTrue(blue(120.0))
        assertFalse(blue(95.0))
        assertTrue(FieldRules.isHubActive(true, true, false, "", 15.0)) // auto
        assertTrue(FieldRules.isHubActive(true, false, true, "", 120.0)) // no data yet
        assertFalse(FieldRules.isHubActive(null, false, true, "R", 120.0)) // no alliance
    }
}
