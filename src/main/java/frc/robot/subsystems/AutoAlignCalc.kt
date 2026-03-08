/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.Constants
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

object AutoAlignCalc {
    // Returns (lateralError, forwardError): how far the robot is from its goal position,
    // in the robot's own coordinate frame (x = left/right, z = forward/backward).
    //
    // The goal is NOT at the tag itself — it's offset to one side (where the scoring
    // mechanism lines up). Those offsets are defined relative to the TAG'S facing
    // direction, not the robot's. When the tag is rotated, the goal rotates with it.
    //
    // Limelight robot-space coordinates (looking down from above):
    //
    //              [Tag face — AprilTag on wall]
    //                    ↑  Z+ of tag
    //         goalX ←───·───→  +X of tag
    //
    //   Robot →  curPose.z meters away, curPose.x meters to the side
    //
    // To find where the goal is from the robot's perspective, we rotate the goal
    // offset by the tag's yaw. This is standard 2D rotation math:
    //   rotated_x = cos(yaw)·goalX + sin(yaw)·goalZ
    //   rotated_z = cos(yaw)·goalZ - sin(yaw)·goalX  (sign flipped for z axis direction)
    //
    // Note: curPose.x is negated because Limelight's X+ points right but our lateral
    // error convention treats rightward tag offset as negative lateral error.
    private fun calculateGoalError(
        currentPose: Pose3d,
        goalX: Double,
        goalZ: Double,
    ): Pair<Double, Double> {
        val yaw = currentPose.rotation.y
        val lateralError = -currentPose.x - (sin(yaw) * goalZ + cos(yaw) * goalX)
        val forwardError = currentPose.z - (cos(yaw) * goalZ + sin(yaw) * goalX)
        return Pair(lateralError, forwardError)
    }

    // Returns a speed multiplier [0.0, 1.0] based on how far the robot is from its goal.
    //
    // Outside FINE_ALIGN_DEADZONE (> 1 m away): full speed (1.0)
    // Inside FINE_ALIGN_DEADZONE:               speed scales down proportionally
    //
    // The sqrt() is applied so the robot decelerates gradually rather than linearly —
    // without it, the speed drops sharply near the target, which feels jerky.
    // With sqrt: at 50% of the distance → 71% speed (√0.5 ≈ 0.71), not 50%.
    private fun calculateTranslationScale(
        lateralError: Double,
        forwardError: Double,
    ): Double {
        val distance = sqrt(lateralError.pow(2) + forwardError.pow(2))
        val ratio =
            if (distance > Constants.AlignConstants.FINE_ALIGN_DEADZONE) {
                1.0
            } else {
                distance / Constants.AlignConstants.FINE_ALIGN_DEADZONE
            }
        return sqrt(ratio)
    }

    // Returns (lateralSpeed, forwardSpeed) scaled so the larger axis has magnitude 1.0
    // and the smaller axis is proportionally reduced — keeping the robot moving in the
    // right direction without either axis exceeding the maximum.
    //
    // Example: lateralError = 0.6 m, forwardError = 0.8 m
    //   → forwardSpeed = 1.0, lateralSpeed = 0.6/0.8 = 0.75
    //   → The robot moves mostly forward, slightly left — correct direction.
    //
    // Signs are encoded into the returned speeds so that:
    //   positive forwardSpeed → robot drives forward (toward tag)
    //   negative lateralSpeed → robot drives right
    // (This matches the negation applied in getAlignSpeeds' ChassisSpeeds output.)
    private fun normalizeTranslation(
        lateralError: Double,
        forwardError: Double,
    ): Pair<Double, Double> {
        val absLateral = abs(lateralError)
        val absForward = abs(forwardError)
        var lateralSpeed = if (absLateral >= absForward) 1.0 else absLateral / absForward
        var forwardSpeed = if (absForward >= absLateral) 1.0 else absForward / absLateral
        if (forwardError < 0.0) forwardSpeed = -forwardSpeed
        if (lateralError > 0.0) lateralSpeed = -lateralSpeed
        return Pair(lateralSpeed, forwardSpeed)
    }

    // Zeroes out a speed axis if the corresponding error is within the deadzone threshold.
    // Each axis is checked against the ORIGINAL error (not the normalized speed) because
    // the deadzone is a physical distance, not a speed ratio.
    //
    // This prevents the robot from twitching back and forth with tiny corrections when
    // it is already close enough to the goal on one axis.
    private fun applyTranslationDeadzone(
        lateralSpeed: Double,
        forwardSpeed: Double,
        lateralError: Double,
        forwardError: Double,
    ): Pair<Double, Double> {
        val deadzone = Constants.AlignConstants.ALIGN_DEADZONE
        return Pair(
            if (abs(lateralError) < deadzone) 0.0 else lateralSpeed,
            if (abs(forwardError) < deadzone) 0.0 else forwardSpeed,
        )
    }

    // Returns the angular velocity (rad/s) the robot should rotate at to face the tag.
    // Positive yaw → tag rotated CCW relative to robot → robot rotates CW (negative omega).
    //
    // Speed zones (mirrors the translation fine-align pattern):
    //
    //   |yaw| > 20° (FINE_ALIGN_ROT_DEADZONE) → full angular speed
    //   5° < |yaw| < 20°                       → proportional slowdown (sqrt curve)
    //   |yaw| < 5°  (ALIGN_ROT_DEADZONE)       → stop rotating
    //
    // Bug fix: Inside the fine-align zone, the original code used:
    //   max(MAX_ANGULAR_SPEED, abs(yaw) / FINE_ALIGN_ROT_DEADZONE)
    //
    // Since MAX_ANGULAR_SPEED = 1.0 and abs(yaw)/FINE_ALIGN_ROT_DEADZONE is always < 1.0
    // inside the zone, max() always returned 1.0 — FULL SPEED, every time.
    //
    // Think of driving toward a stop sign: you want to slow down as you approach,
    // not drive at top speed until you slam on the brakes. The fix uses min() instead,
    // so the robot actually slows its rotation as alignment improves:
    //
    //   At yaw = 20°: min(1.0, 20/20) = 1.0  → full speed
    //   At yaw = 10°: min(1.0, 10/20) = 0.5  → half speed
    //   At yaw = 5°:  min(1.0, 5/20)  = 0.25 → quarter speed → then STOP
    private fun calculateRotationOutput(yaw: Double): Double {
        if (abs(yaw) < Constants.AlignConstants.ALIGN_ROT_DEADZONE) return 0.0
        val rotationDirection = -(yaw / abs(yaw))
        val rotationScale =
            if (abs(yaw) > Constants.AlignConstants.FINE_ALIGN_ROT_DEADZONE) {
                1.0
            } else {
                abs(yaw) / Constants.AlignConstants.FINE_ALIGN_ROT_DEADZONE
            }
        return rotationDirection * sqrt(rotationScale) * Constants.AlignConstants.MAX_ANGULAR_SPEED
    }

    // Calculates the chassis speeds needed to move the robot to its scoring position
    // relative to the nearest visible AprilTag.
    //
    // ChassisSpeeds axes (WPILib convention, robot-relative):
    //   vx → forward/backward  (positive = forward)
    //   vy → left/right        (positive = left)
    //   omega → rotation       (positive = counter-clockwise)
    fun getAlignSpeeds(
        goalX: Double,
        goalZ: Double,
        curPose: Pose3d,
    ): ChassisSpeeds {
        val (lateralError, forwardError) = calculateGoalError(curPose, goalX, goalZ)
        val translationScale = calculateTranslationScale(lateralError, forwardError)
        val (lateralSpeed, forwardSpeed) = normalizeTranslation(lateralError, forwardError)
        val (adjustedLateral, adjustedForward) =
            applyTranslationDeadzone(
                lateralSpeed,
                forwardSpeed,
                lateralError,
                forwardError,
            )
        return ChassisSpeeds(
            adjustedForward * translationScale * Constants.AlignConstants.MAX_SPEED,
            -adjustedLateral * translationScale * Constants.AlignConstants.MAX_SPEED,
            calculateRotationOutput(curPose.rotation.y),
        )
    }
}
