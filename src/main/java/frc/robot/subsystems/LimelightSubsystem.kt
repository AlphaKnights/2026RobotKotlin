/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.math.geometry.Pose3d
import frc.robot.Constants
import frc.robot.LimelightHelpers.LimelightResults
import frc.robot.interfaces.LimelightService
import frc.robot.interfaces.PoseProvider
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlin.math.abs
import kotlin.math.absoluteValue

object LimelightSubsystem : PoseProvider {
    private val mutex = Mutex()
    private var _tagPose: Pose3d? = null
    override val tagPose: Pose3d?
        get() = runBlocking { mutex.withLock { _tagPose } }
    var coroutineScope =
        CoroutineScope(
            Dispatchers.Default + SupervisorJob(),
        )

    var limelightService: LimelightService = HttpLimelightService

    fun startPolling() {
        coroutineScope.launch {
            while (true) {
                setTagPose(updateTagPosition())
                delay(
                    Constants.LimelightConstants.POLLING_RATE,
                )
            }
        }
    }

    suspend fun updateTagPosition(): Pose3d? {
        kotlin
            .runCatching {
                limelightService
                    .fetchResults()
                    ?.let(::parseJson)
                    ?.targets_Fiducials
                    ?.firstOrNull()
                    ?.targetPose_RobotSpace
            }.onSuccess { pose ->
                return pose
            }.onFailure {
                println("Failed to update tag position: $it")
                return null
            }
        return null
    }

    suspend fun setTagPose(pose: Pose3d?) {
        mutex.withLock {
            _tagPose = pose
        }
    }

    fun isAligned(): Boolean {
        val pose = tagPose ?: return false

        return (
            isWithinLeftPosition(pose) ||
                isWithinRightPosition(pose)
        ) &&
            isRotationAligned(pose)
    }

    // Bug fix: The original isAligned() returned true whenever the robot was at
    // EITHER the left OR the right scoring position. This means if a command is
    // trying to align RIGHT but the robot happens to already be sitting at the LEFT
    // position, isAligned() fires immediately — the command "succeeds" without the
    // robot going anywhere useful.
    //
    // This overload checks only the position that was actually commanded.
    // The original no-argument version is kept for dashboard indicators and general use.
    fun isAligned(direction: Constants.AlignDirection): Boolean {
        val pose = tagPose ?: return false
        return when (direction) {
            Constants.AlignDirection.LEFT -> isWithinLeftPosition(pose) && isRotationAligned(pose)
            Constants.AlignDirection.RIGHT -> isWithinRightPosition(pose) && isRotationAligned(pose)
        }
    }

    private fun isWithinLeftPosition(pose: Pose3d): Boolean {
        val translation = pose.translation
        return abs(
            translation.x +
                Constants.AlignConstants.LEFT_X_OFFSET,
        ) <=
            Constants.AlignConstants.ALIGN_DEADZONE &&
            abs(
                translation.z -
                    Constants.AlignConstants.LEFT_Z_OFFSET,
            ) <=
            Constants.AlignConstants.ALIGN_DEADZONE
    }

    private fun isWithinRightPosition(pose: Pose3d): Boolean {
        val translation = pose.translation
        return (
            abs(
                translation.x +
                    Constants.AlignConstants.RIGHT_X_OFFSET,
            ) <=
                Constants.AlignConstants.ALIGN_DEADZONE &&
                abs(
                    translation.z -
                        Constants.AlignConstants.RIGHT_Z_OFFSET,
                ) <=
                Constants.AlignConstants.ALIGN_DEADZONE
        )
    }

    // Bug fix: This was checking pose.rotation.z (roll — the tag tilting sideways like a
    // leaning picture frame). The axis we actually care about is yaw — how much the tag
    // is turned left or right — which the Limelight reports in rotation.y.
    //
    // Limelight targetPose_RobotSpace rotation axes:
    //   rotation.x → pitch  (tag nodding forward/backward)
    //   rotation.y → YAW    (tag turning left/right)  ← this is alignment rotation
    //   rotation.z → roll   (tag tilting sideways)
    //
    // AutoAlignCalc already reads rotation.y for driving. This check must use the
    // same axis, or the robot drives using yaw but declares victory based on roll —
    // two different things that will rarely agree.
    private fun isRotationAligned(pose: Pose3d): Boolean =
        pose.rotation.y.absoluteValue <= Constants.AlignConstants.ALIGN_ROT_DEADZONE

    fun parseJson(json: String?): LimelightResults? {
        val checkedJson: String = json ?: return null
        val mapper: ObjectMapper =
            ObjectMapper().configure(
                DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES,
                false,
            )

        return try {
            mapper.readValue(
                checkedJson,
                LimelightResults::class.java,
            )
        } catch (e: Exception) {
            println("Failed to parse JSON: $e")
            null
        }
    }
}
