/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants.FieldConstants
import frc.robot.Constants.VisionConstants
import frc.robot.LimelightHelpers
import frc.robot.util.Tunables

/**
 * Feeds Limelight 4 AprilTag poses into the drivetrain's pose estimator.
 *
 * Nothing aims off the camera directly. Wheel odometry + the Pigeon update every 20 ms and the
 * Limelight only corrects their drift, so 15 fps (or less) is plenty and the camera can run cool.
 */
object LimelightVision {
    private const val NAME = VisionConstants.LIMELIGHT_NAME

    // MegaTag2 gets its heading from us, so never let it "correct" heading.
    private const val IGNORE_HEADING = 9_999_999.0

    // Indices into the Limelight "hw" array: [cpu temp C, cpu %, ram %, fps].
    private const val HW_TEMP_INDEX = 0
    private const val HW_FPS_INDEX = 3

    private val mt2Limits =
        VisionMath.Mt2Limits(
            maxYawRateDegPerSec = VisionConstants.MAX_YAW_RATE_DEG_PER_SEC,
            maxTagDistanceM = VisionConstants.MAX_TAG_DISTANCE_M,
            fieldLength = FieldConstants.FIELD_LENGTH,
            fieldWidth = FieldConstants.FIELD_WIDTH,
        )

    private var lastMt2Timestamp = Double.NaN
    private var lastMt1Timestamp = Double.NaN
    private var lastAcceptedTime = Double.NEGATIVE_INFINITY
    private var lastThrottle = -1

    /**
     * Call once per loop, right after updating odometry.
     *
     * @return while disabled, a MegaTag1 pose good enough to reset the whole robot pose to
     * (including heading); otherwise null.
     */
    fun update(
        estimator: SwerveDrivePoseEstimator,
        yawRateDegPerSec: Double,
        disabled: Boolean,
    ): Pose2d? {
        setThrottle(if (disabled) VisionConstants.THROTTLE_DISABLED else VisionConstants.THROTTLE_ENABLED)

        // MegaTag2 solves position using OUR heading, so send it every loop before reading results.
        LimelightHelpers.SetRobotOrientation(
            NAME,
            estimator.estimatedPosition.rotation.degrees,
            yawRateDegPerSec,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        publishHardware()

        // Kill switch: if vision is making the pose worse, turn it off and drive on odometry alone.
        if (!Tunables.flag("Vision/Enabled", true)) {
            SmartDashboard.putString("Vision/Status", "off (Vision/Enabled is false): odometry only")
            return null
        }

        if (disabled) return seedFromMegaTag1()
        fuseMegaTag2(estimator, yawRateDegPerSec)
        return null
    }

    private fun fuseMegaTag2(
        estimator: SwerveDrivePoseEstimator,
        yawRateDegPerSec: Double,
    ) {
        val estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(NAME)
        SmartDashboard.putNumber("Vision/TagCount", (estimate?.tagCount ?: 0).toDouble())
        SmartDashboard.putNumber("Vision/SecondsSinceUpdate", Timer.getFPGATimestamp() - lastAcceptedTime)
        if (estimate == null) {
            SmartDashboard.putString("Vision/Status", "no data from '$NAME': check its name and network")
            return
        }

        val isNewFrame = estimate.timestampSeconds != lastMt2Timestamp
        lastMt2Timestamp = estimate.timestampSeconds
        // The camera runs slower than the robot loop, so most loops have no new frame. Not news.
        if (!isNewFrame) return

        val sample =
            VisionMath.Mt2Sample(
                tagCount = estimate.tagCount,
                avgTagDistM = estimate.avgTagDist,
                yawRateDegPerSec = yawRateDegPerSec,
                x = estimate.pose.x,
                y = estimate.pose.y,
                isNewFrame = true,
            )
        val rejected = VisionMath.rejectReason(sample, mt2Limits)
        if (rejected != null) {
            SmartDashboard.putString("Vision/Status", "rejected: $rejected")
            return
        }

        val xyStdDev =
            VisionMath.xyStdDev(
                estimate.avgTagDist,
                estimate.tagCount,
                VisionConstants.XY_STD_DEV_PER_M2,
                VisionConstants.MIN_XY_STD_DEV,
            )
        estimator.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, IGNORE_HEADING),
        )
        lastAcceptedTime = Timer.getFPGATimestamp()
        SmartDashboard.putString(
            "Vision/Status",
            "accepted: %d tag(s) at %.1f m, trust +/-%.2f m".format(estimate.tagCount, estimate.avgTagDist, xyStdDev),
        )
    }

    /** MegaTag1 measures heading too, so with 2+ tags it can tell us which way the robot faces. */
    private fun seedFromMegaTag1(): Pose2d? {
        val estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(NAME)
        if (estimate == null) {
            SmartDashboard.putString("Vision/Status", "disabled, no data from '$NAME': check its name and network")
            return null
        }
        val isNewFrame = estimate.timestampSeconds != lastMt1Timestamp
        lastMt1Timestamp = estimate.timestampSeconds
        if (!isNewFrame) return null

        val onField =
            VisionMath.onField(
                estimate.pose.x,
                estimate.pose.y,
                FieldConstants.FIELD_LENGTH,
                FieldConstants.FIELD_WIDTH,
            )
        val problem =
            when {
                estimate.tagCount < VisionConstants.SEED_MIN_TAGS ->
                    "sees ${estimate.tagCount} tag(s), needs ${VisionConstants.SEED_MIN_TAGS} to set heading"
                estimate.avgTagDist > VisionConstants.SEED_MAX_TAG_DISTANCE_M ->
                    "tags too far to set heading (%.1f m)".format(estimate.avgTagDist)
                !onField -> "pose off the field: check camera pose"
                else -> null
            }
        if (problem != null) {
            SmartDashboard.putString("Vision/Status", "disabled, waiting to seed: $problem")
            return null
        }
        SmartDashboard.putString("Vision/Status", "disabled, seeded pose + heading from ${estimate.tagCount} tags")
        DataLogManager.log("Vision: seeded pose from MegaTag1 (${estimate.tagCount} tags): ${estimate.pose}")
        return estimate.pose
    }

    /** Skip frames while disabled so the Limelight doesn't cook itself sitting in the queue. */
    private fun setThrottle(framesToSkip: Int) {
        if (framesToSkip == lastThrottle) return
        LimelightHelpers.setLimelightNTDouble(NAME, "throttle_set", framesToSkip.toDouble())
        lastThrottle = framesToSkip
    }

    private fun publishHardware() {
        val hw = LimelightHelpers.getLimelightNTDoubleArray(NAME, "hw")
        if (hw.size > HW_FPS_INDEX) {
            SmartDashboard.putNumber("Vision/LimelightTempC", hw[HW_TEMP_INDEX])
            SmartDashboard.putNumber("Vision/LimelightFps", hw[HW_FPS_INDEX])
        }
    }
}
