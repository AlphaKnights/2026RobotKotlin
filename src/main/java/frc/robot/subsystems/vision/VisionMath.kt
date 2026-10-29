/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.vision

import kotlin.math.abs
import kotlin.math.max

/** Rules for trusting a Limelight pose. No WPILib imports, so these are unit tested. */
object VisionMath {
    private const val OFF_FIELD_MARGIN_M = 0.5

    /** Std dev (m) for a vision XY measurement: grows with tag distance, shrinks with more tags. */
    fun xyStdDev(
        avgTagDistM: Double,
        tagCount: Int,
        perMeterSquared: Double,
        minimum: Double,
    ): Double = max(minimum, perMeterSquared * avgTagDistM * avgTagDistM / max(1, tagCount))

    /** True if the point is on the field (with a little margin for bumpers against the wall). */
    fun onField(
        x: Double,
        y: Double,
        fieldLength: Double,
        fieldWidth: Double,
    ): Boolean =
        x >= -OFF_FIELD_MARGIN_M &&
            x <= fieldLength + OFF_FIELD_MARGIN_M &&
            y >= -OFF_FIELD_MARGIN_M &&
            y <= fieldWidth + OFF_FIELD_MARGIN_M

    /** Everything needed to judge one MegaTag2 result. */
    data class Mt2Sample(
        val tagCount: Int,
        val avgTagDistM: Double,
        val yawRateDegPerSec: Double,
        val x: Double,
        val y: Double,
        val isNewFrame: Boolean,
    )

    data class Mt2Limits(
        val maxYawRateDegPerSec: Double,
        val maxTagDistanceM: Double,
        val fieldLength: Double,
        val fieldWidth: Double,
    )

    fun acceptMegaTag2(
        sample: Mt2Sample,
        limits: Mt2Limits,
    ): Boolean = rejectReason(sample, limits) == null

    /** Why a MegaTag2 result was thrown out, in plain words, or null if it is good to use. */
    fun rejectReason(
        sample: Mt2Sample,
        limits: Mt2Limits,
    ): String? =
        when {
            !sample.isNewFrame -> "same frame as last time"
            sample.tagCount <= 0 -> "no tags in view"
            abs(sample.yawRateDegPerSec) > limits.maxYawRateDegPerSec ->
                "spinning too fast (%.0f deg/s)".format(abs(sample.yawRateDegPerSec))
            sample.avgTagDistM > limits.maxTagDistanceM -> "tags too far (%.1f m)".format(sample.avgTagDistM)
            !onField(sample.x, sample.y, limits.fieldLength, limits.fieldWidth) ->
                "pose off the field (x=%.1f, y=%.1f): check camera pose".format(sample.x, sample.y)
            else -> null
        }
}
