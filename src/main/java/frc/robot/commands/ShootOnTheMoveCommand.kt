/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.ShotConstants
import frc.robot.subsystems.DeliverySubsystem
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.StorageSubsystem
import frc.robot.subsystems.aiming.FeedGate
import frc.robot.subsystems.aiming.HubTargeting
import frc.robot.subsystems.aiming.ShotSolution
import frc.robot.util.Tunables
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Hold to shoot while driving. The driver keeps translation; the robot handles the rest:
 *  1. turns so the shooter points at the "virtual target" (the HUB shifted against our velocity,
 *     because FUEL keeps the robot's velocity after it leaves),
 *  2. sets the flywheel from the distance to that virtual target (so driving toward the HUB needs
 *     less speed and driving away needs more),
 *  3. runs the indexer only while in range, legal (G407), aimed and at speed.
 *
 * Debugging: "Shot/BlockedBy" on the dashboard says, in words, the one thing stopping the shot.
 * "Shot/DryRun" runs everything except the indexer. Gains are live under "Tune/".
 *
 * Tuning: turn on "Shot/TuningMode", set "Shot/TuningRps", hold this from a standing spot and adjust
 * until it scores. Write down "Shot/DistanceM" and the RPS, and put the pairs in
 * ShotConstants.FLYWHEEL_RPS_BY_DISTANCE.
 *
 * Pass `{ 0.0 }` for both inputs to aim and shoot in place (used by autos).
 */
class ShootOnTheMoveCommand(
    private val driverForward: () -> Double,
    private val driverLeft: () -> Double,
) : Command() {
    private val headingController =
        PIDController(ShotConstants.AIM_P, 0.0, ShotConstants.AIM_D).apply {
            enableContinuousInput(-PI, PI)
        }
    private var feedDebouncer = newDebouncer()
    private var wasFiring = false

    init {
        addRequirements(DriveSubsystem, DeliverySubsystem, StorageSubsystem)
    }

    override fun initialize() {
        headingController.reset()
        feedDebouncer = newDebouncer()
        wasFiring = false
    }

    override fun execute() {
        val pose = DriveSubsystem.getPose()
        val shot = HubTargeting.solve(pose, DriveSubsystem.getCurrentSpeeds())

        // 1. Rotation: feedforward for how fast the aim point is sweeping, plus feedback on the error.
        headingController.setPID(
            Tunables.number("Tune/AimP", ShotConstants.AIM_P),
            0.0,
            Tunables.number("Tune/AimD", ShotConstants.AIM_D),
        )
        val maxOmega = Tunables.number("Tune/MaxAimOmega", ShotConstants.MAX_AIM_OMEGA)
        val omega =
            MathUtil.clamp(
                shot.headingFeedforwardRadPerSec +
                    headingController.calculate(pose.rotation.radians, shot.robotHeadingSetpointRad),
                -maxOmega,
                maxOmega,
            )
        val headingError = MathUtil.angleModulus(shot.robotHeadingSetpointRad - pose.rotation.radians)

        // 2. Translation stays with the driver, in the same driver-relative frame as DriveCommand.
        val (vx, vy) = cappedDriverTranslation()
        DriveSubsystem.drive(ChassisSpeeds(vx, vy, omega), fieldRelative = true)

        // 3. Flywheel.
        val tuningRps = HubTargeting.tuningRps()
        val targetRps = tuningRps ?: shot.flywheelRps
        DeliverySubsystem.setVelocity(targetRps)

        // 4. Indexer: feed only when nothing is blocking the shot.
        val blockedBy = feedBlockedBy(pose, shot, headingError, targetRps, tuning = tuningRps != null)
        val firing = feedDebouncer.calculate(blockedBy == null)
        if (firing) {
            StorageSubsystem.roll(ShotConstants.FEED_SPEED)
        } else {
            StorageSubsystem.rollerstop()
        }
        if (firing && !wasFiring) logShot(shot, targetRps)
        wasFiring = firing

        val status = blockedBy ?: if (firing) HubTargeting.FIRING else "settling (ready for ${FEED_DEBOUNCE_MS} ms)"
        HubTargeting.publishShotState(
            headingError,
            aimed = abs(headingError) <= shot.aimToleranceRad,
            atSpeed = abs(DeliverySubsystem.velocityRps() - targetRps) <= flywheelToleranceRps(),
            status = status,
        )
    }

    /** The one thing stopping the shot, or null when it's good. See FeedGate for the order checked. */
    private fun feedBlockedBy(
        pose: Pose2d,
        shot: ShotSolution,
        headingError: Double,
        targetRps: Double,
        tuning: Boolean,
    ): String? =
        FeedGate.blockedBy(
            FeedGate.Inputs(
                distanceM = shot.distanceM,
                inRange = shot.inRange || tuning,
                tableMinM = HubTargeting.tableMinM,
                tableMaxM = HubTargeting.tableMaxM,
                inAllianceZone = HubTargeting.inAllianceZone(pose, allowPracticeOverride = true),
                headingErrorRad = headingError,
                aimToleranceRad = shot.aimToleranceRad,
                flywheelRps = DeliverySubsystem.velocityRps(),
                targetRps = targetRps,
                flywheelToleranceRps = flywheelToleranceRps(),
                dryRun = Tunables.flag("Shot/DryRun", false),
            ),
        )

    private fun flywheelToleranceRps() =
        Tunables.number("Tune/FlywheelToleranceRps", ShotConstants.FLYWHEEL_TOLERANCE_RPS)

    /** One log line per burst: after practice, AdvantageScope shows exactly what each shot was. */
    private fun logShot(
        shot: ShotSolution,
        targetRps: Double,
    ) {
        val speeds = DriveSubsystem.getCurrentSpeeds()
        DataLogManager.log(
            "Shot: firing at %.2f m, %.1f rps, tof %.2f s, robot %.2f m/s".format(
                shot.distanceM,
                targetRps,
                shot.timeOfFlightSec,
                hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            ),
        )
    }

    /** Driver translation in m/s, scaled down so the robot never shoots faster than the cap. */
    private fun cappedDriverTranslation(): Pair<Double, Double> {
        val vx = driverForward() * DriveConstants.MAX_METERS_PER_SECOND
        val vy = driverLeft() * DriveConstants.MAX_METERS_PER_SECOND
        val speed = hypot(vx, vy)
        val cap = Tunables.number("Tune/MaxDriveSpeedWhileShooting", ShotConstants.MAX_DRIVE_SPEED_WHILE_SHOOTING)
        return if (speed > cap) Pair(vx * cap / speed, vy * cap / speed) else Pair(vx, vy)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        StorageSubsystem.rollerstop()
        DeliverySubsystem.stop()
        DriveSubsystem.drive(ChassisSpeeds(), fieldRelative = false)
        HubTargeting.publishShotState(0.0, aimed = false, atSpeed = false, status = "not shooting (B released)")
    }

    private fun newDebouncer() = Debouncer(ShotConstants.FEED_DEBOUNCE_SEC, Debouncer.DebounceType.kRising)

    private companion object {
        const val MILLIS_PER_SECOND = 1000.0
        val FEED_DEBOUNCE_MS = (ShotConstants.FEED_DEBOUNCE_SEC * MILLIS_PER_SECOND).toInt()
    }
}
