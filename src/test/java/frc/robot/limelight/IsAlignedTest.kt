/*
 * (C) 2025 Galvaknights
 */
package frc.robot.limelight

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import frc.robot.Constants.AlignConstants
import frc.robot.Constants.AlignDirection
import frc.robot.subsystems.LimelightSubsystem
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

// Coordinate note: isWithinLeftPosition checks abs(x + LEFT_X_OFFSET) ≤ deadzone.
// Since LEFT_X_OFFSET = -0.1625, the LEFT scoring position has x ≈ +0.1625.
// Since RIGHT_X_OFFSET = +0.1625, the RIGHT scoring position has x ≈ -0.1625.
// Test poses are labeled by which isWithin*Position check they satisfy.
internal class IsAlignedTest {
    @Test
    fun `isAligned returns true when april tag on different sides of robot`() =
        runTest {
            // x = LEFT_X_OFFSET = -0.1625 → satisfies isWithinRightPosition (abs(-0.1625+0.1625)=0)
            // Use 0.0 rotation to avoid floating-point round-trip error through quaternion math
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    0.0,
                    AlignConstants.LEFT_Z_OFFSET,
                    Rotation3d(0.0, 0.0, 0.0),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(AlignDirection.RIGHT),
            )

            // x = RIGHT_X_OFFSET = 0.1625 → satisfies isWithinLeftPosition (abs(0.1625-0.1625)=0)
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    0.0,
                    AlignConstants.RIGHT_Z_OFFSET,
                    Rotation3d(0.0, 0.0, 0.0),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(AlignDirection.LEFT),
            )
        }

    @Test
    fun `isAligned returns false when the x position of the tag is beyond the range`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    AlignConstants.LEFT_Z_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                        0.0,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.LEFT),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    AlignConstants.RIGHT_Z_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                        0.0,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.RIGHT),
            )
        }

    @Test
    fun `isAligned returns false when the z position of the tag is beyond the range`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    AlignConstants.LEFT_Z_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                        0.0,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.LEFT),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    AlignConstants.RIGHT_Z_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                        0.0,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.RIGHT),
            )
        }

    @Test
    fun `isAligned returns false when the rotation of the tag is beyond the range`() =
        runTest {
            // z=0 means neither position check passes, so both directions return false
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    AlignConstants.LEFT_Z_OFFSET,
                    0.0,
                    Rotation3d(
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE +
                            0.1,
                        0.0,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.LEFT),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    AlignConstants.RIGHT_Z_OFFSET,
                    0.0,
                    Rotation3d(
                        0.0,
                        -AlignConstants.ALIGN_ROT_DEADZONE -
                            0.1,
                        0.0,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.RIGHT),
            )
        }

    @Test
    fun `isAligned returns true when the rotation of the tag is within the range`() =
        runTest {
            // x = LEFT_X_OFFSET = -0.1625 → satisfies isWithinRightPosition
            // Use half the deadzone to stay well clear of the boundary and avoid
            // floating-point round-trip error through quaternion reconstruction.
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    0.0,
                    AlignConstants.LEFT_Z_OFFSET,
                    Rotation3d(0.0, AlignConstants.ALIGN_ROT_DEADZONE / 2.0, 0.0),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(AlignDirection.RIGHT),
            )

            // x = RIGHT_X_OFFSET = 0.1625 → satisfies isWithinLeftPosition
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    0.0,
                    AlignConstants.RIGHT_Z_OFFSET,
                    Rotation3d(0.0, -AlignConstants.ALIGN_ROT_DEADZONE / 2.0, 0.0),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(AlignDirection.LEFT),
            )
        }

    @Test
    fun `isAligned RIGHT returns false when robot is at LEFT scoring position`() =
        runTest {
            // LEFT scoring position: x ≈ +0.1625 (= RIGHT_X_OFFSET = -LEFT_X_OFFSET)
            // isWithinLeftPosition passes; isWithinRightPosition fails → isAligned(RIGHT) = false
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    0.0,
                    AlignConstants.LEFT_Z_OFFSET,
                    Rotation3d(0.0, 0.0, 0.0),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.RIGHT),
            )
        }

    @Test
    fun `isAligned LEFT returns false when robot is at RIGHT scoring position`() =
        runTest {
            // RIGHT scoring position: x ≈ -0.1625 (= LEFT_X_OFFSET = -RIGHT_X_OFFSET)
            // isWithinRightPosition passes; isWithinLeftPosition fails → isAligned(LEFT) = false
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    0.0,
                    AlignConstants.RIGHT_Z_OFFSET,
                    Rotation3d(0.0, 0.0, 0.0),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(AlignDirection.LEFT),
            )
        }
}
