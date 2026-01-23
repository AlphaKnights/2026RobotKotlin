/*
 * (C) 2025 Galvaknights
 */
package frc.robot.limelight

import frc.robot.subsystems.LimelightSubsystem
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

class ParseJsonTest {
    @Test
    fun `parseJson returns null when Limelight has no target`() {
        val result = LimelightSubsystem.parseJson(null)
        assertNull(result)
    }

    @Test
    fun `parseJson returns null when a bad string is given`() {
        val result = LimelightSubsystem.parseJson("bad string")
        assertNull(result)
    }

    @Test
    @Suppress("LongMethod")
    fun `parseJson returns a valid Pose3d when a valid pose is given`() {
        val validJson = """
            {
            "Classifier": [],
            "Detector": [],
            "Fiducial": [
            {
                "fID": 2,
                "fam": "16H5C",
                "pts": [],
                "skew": [],
                "t6c_ts": [
                0.33247368976801916,
                -0.05672695778305914,
                -2.5042031405987144,
                -4.680849607956358,
                -5.171154989721864,
                4.528697946312339
                ],
                "t6r_fs": [
                4.738896418276903,
                -1.5926603672041666,
                0.5194469577830592,
                4.522658587661256,
                4.258580454853879,
                5.5236539893713275
                ],
                "t6r_ts": [
                0.33247368976801916,
                -0.05672695778305914,
                -2.5042031405987144,
                -4.680849607956358,
                -5.171154989721864,
                4.528697946312339
                ],
                "t6t_cs": [
                -0.09991902572799474,
                -0.1234042720218289,
                2.5218203039582496,
                4.278368708252767,
                5.508508005282244,
                -4.1112864453027775
                ],
                "t6t_rs": [
                -0.09991902572799474,
                -0.1234042720218289,
                2.5218203039582496,
                4.278368708252767,
                5.508508005282244,
                -4.1112864453027775
                ],
                "ta": 0.005711808800697327,
                "tx": -2.0525293350219727,
                "txp": 149.4874725341797,
                "ty": 2.7294836044311523,
                "typ": 107.14710235595703
            }
            ],
            "Retro": [],
            "pID": 0,
            "tl": 19.78130340576172,
            "ts": 3284447.910569,
            "v": 1
        }
        """

        val result = LimelightSubsystem.parseJson(validJson)
        assert(result != null)
        assertEquals(
            -0.09991902572799474,
            result!!
                .targets_Fiducials
                ?.get(0)
                ?.targetPose_RobotSpace
                ?.x,
        )
        assertEquals(
            -0.1234042720218289,
            result.targets_Fiducials
                ?.get(0)
                ?.targetPose_RobotSpace
                ?.y,
        )
        assertEquals(
            2.5218203039582496,
            result.targets_Fiducials
                ?.get(0)
                ?.targetPose_RobotSpace
                ?.z,
        )
    }

    @Test
    fun `parseJson returns an empty limelight result when a valid empty string is given`() {
        val validEmptyJson = """
            {
            "Classifier": [],
            "Detector": [],
            "Fiducial": [],
            "Retro": [],
            "pID": 0,
            "tl": 19.78130340576172,
            "ts": 3284447.910569,
            "v": 1
        }
        """

        val result = LimelightSubsystem.parseJson(validEmptyJson)
        assert(result != null)
        assertEquals(0, result!!.targets_Fiducials?.size)
    }
}
