/*
 * (C) 2025 Galvaknights
 */
package frc.robot.vision

import frc.robot.subsystems.vision.VisionMath
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

internal class VisionMathTest {
    private val limits =
        VisionMath.Mt2Limits(
            maxYawRateDegPerSec = 360.0,
            maxTagDistanceM = 6.0,
            fieldLength = 16.541,
            fieldWidth = 8.069,
        )

    private val good =
        VisionMath.Mt2Sample(
            tagCount = 1,
            avgTagDistM = 2.5,
            yawRateDegPerSec = 40.0,
            x = 3.0,
            y = 4.0,
            isNewFrame = true,
        )

    @Test
    fun `a normal sample is accepted`() {
        assertTrue(VisionMath.acceptMegaTag2(good, limits))
    }

    @Test
    fun `bad samples are rejected`() {
        assertFalse(VisionMath.acceptMegaTag2(good.copy(isNewFrame = false), limits), "same frame twice")
        assertFalse(VisionMath.acceptMegaTag2(good.copy(tagCount = 0), limits), "no tags")
        assertFalse(VisionMath.acceptMegaTag2(good.copy(yawRateDegPerSec = -500.0), limits), "spinning")
        assertFalse(VisionMath.acceptMegaTag2(good.copy(avgTagDistM = 7.0), limits), "tags too far")
        assertFalse(VisionMath.acceptMegaTag2(good.copy(x = -2.0), limits), "off field")
        assertFalse(VisionMath.acceptMegaTag2(good.copy(y = 9.0), limits), "off field")
    }

    @Test
    fun `each rejection says why in words`() {
        assertEquals(null, VisionMath.rejectReason(good, limits))
        assertEquals("same frame as last time", VisionMath.rejectReason(good.copy(isNewFrame = false), limits))
        assertEquals("no tags in view", VisionMath.rejectReason(good.copy(tagCount = 0), limits))
        assertEquals(
            "spinning too fast (500 deg/s)",
            VisionMath.rejectReason(good.copy(yawRateDegPerSec = -500.0), limits),
        )
        assertEquals("tags too far (7.0 m)", VisionMath.rejectReason(good.copy(avgTagDistM = 7.0), limits))
        assertTrue(VisionMath.rejectReason(good.copy(x = -2.0), limits)!!.startsWith("pose off the field"))
    }

    @Test
    fun `trust drops with distance and rises with tag count`() {
        val near = VisionMath.xyStdDev(1.0, 1, 0.1, 0.05)
        val far = VisionMath.xyStdDev(4.0, 1, 0.1, 0.05)
        val farTwoTags = VisionMath.xyStdDev(4.0, 2, 0.1, 0.05)
        assertEquals(0.1, near, 1e-9)
        assertEquals(1.6, far, 1e-9)
        assertEquals(0.8, farTwoTags, 1e-9)
        assertEquals(0.05, VisionMath.xyStdDev(0.3, 3, 0.1, 0.05), 1e-9)
        assertEquals(0.4, VisionMath.xyStdDev(2.0, 0, 0.1, 0.05), 1e-9) // never divides by zero
    }
}
