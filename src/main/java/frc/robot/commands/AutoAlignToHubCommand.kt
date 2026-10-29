/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.ShotConstants
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.aiming.HubTargeting
import frc.robot.util.Tunables
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

/**
 * Drives to the "sweet spot": ShotConstants.SWEET_SPOT_DISTANCE from the HUB center, in front of
 * it, with the shooter pointed at it. Uses the fused pose, so it works whether or not a tag is in
 * view at this exact moment.
 *
 * Only starts from inside our ALLIANCE ZONE; from anywhere else a straight line could clip the HUB.
 *
 * @param finishWhenAligned true for autos (end once there). False for a held button: it keeps
 * holding the spot, which also resists being pushed.
 */
class AutoAlignToHubCommand(
    private val finishWhenAligned: Boolean,
) : Command() {
    private val xController = PIDController(ShotConstants.ALIGN_P, 0.0, 0.0)
    private val yController = PIDController(ShotConstants.ALIGN_P, 0.0, 0.0)
    private val headingController =
        PIDController(ShotConstants.AIM_P, 0.0, ShotConstants.AIM_D).apply {
            enableContinuousInput(-PI, PI)
        }
    private var target: Pose2d? = null

    init {
        addRequirements(DriveSubsystem)
    }

    override fun initialize() {
        val pose = DriveSubsystem.getPose()
        // The goal is picked once, here. (The old DriveToArcCommand picked it when the robot booted.)
        target = if (HubTargeting.inAllianceZone(pose)) HubTargeting.sweetSpotPose(pose) else null
        xController.reset()
        yController.reset()
        headingController.reset()
        HubTargeting.showAlignTarget(target)
        val status =
            if (target == null) {
                "not started: robot is not in our alliance zone (x=%.2f). Drive in first.".format(pose.x)
            } else {
                "driving to (%.2f, %.2f)".format(target?.x, target?.y)
            }
        SmartDashboard.putString("Align/Status", status)
        DataLogManager.log("Align: $status")
    }

    override fun execute() {
        val goal = target ?: return
        val pose = DriveSubsystem.getPose()

        // Gains are live under Tune/ so they can be tuned without redeploying.
        val p = Tunables.number("Tune/AlignP", ShotConstants.ALIGN_P)
        xController.p = p
        yController.p = p
        headingController.setPID(
            Tunables.number("Tune/AimP", ShotConstants.AIM_P),
            0.0,
            Tunables.number("Tune/AimD", ShotConstants.AIM_D),
        )
        val maxSpeed = Tunables.number("Tune/AlignMaxSpeed", ShotConstants.ALIGN_MAX_SPEED)
        val maxOmega = Tunables.number("Tune/MaxAimOmega", ShotConstants.MAX_AIM_OMEGA)

        var vx = xController.calculate(pose.x, goal.x)
        var vy = yController.calculate(pose.y, goal.y)
        val speed = hypot(vx, vy)
        if (speed > maxSpeed) {
            vx *= maxSpeed / speed
            vy *= maxSpeed / speed
        }
        val omega =
            MathUtil.clamp(
                headingController.calculate(pose.rotation.radians, goal.rotation.radians),
                -maxOmega,
                maxOmega,
            )

        // vx/vy are in the FIELD frame, so convert with the pose estimator's heading, not the driver frame.
        DriveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.rotation),
            fieldRelative = false,
        )

        val aligned = isAligned(pose, goal)
        SmartDashboard.putBoolean("Align/Aligned", aligned)
        SmartDashboard.putString(
            "Align/Status",
            if (aligned) {
                "aligned"
            } else {
                "driving: %.2f m and %.1f deg to go".format(
                    pose.translation.getDistance(goal.translation),
                    Math.toDegrees(abs(MathUtil.angleModulus(goal.rotation.radians - pose.rotation.radians))),
                )
            },
        )
    }

    private fun isAligned(
        pose: Pose2d,
        goal: Pose2d,
    ): Boolean =
        pose.translation.getDistance(goal.translation) <= ShotConstants.ALIGN_POSITION_TOLERANCE &&
            abs(MathUtil.angleModulus(goal.rotation.radians - pose.rotation.radians)) <=
            Math.toRadians(ShotConstants.ALIGN_HEADING_TOLERANCE_DEG)

    override fun isFinished(): Boolean {
        val goal = target ?: return true
        return finishWhenAligned && isAligned(DriveSubsystem.getPose(), goal)
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.drive(ChassisSpeeds(), fieldRelative = false)
        HubTargeting.showAlignTarget(null)
        SmartDashboard.putBoolean("Align/Aligned", false)
    }
}
