/*
 * (C) 2025 Galvaknights
 */
package frc.robot.interfaces

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

interface SwerveModule : Sendable {
    override fun initSendable(builder: SendableBuilder?) {
        null
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    fun getPosition(): SwerveModulePosition

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    fun getState(): SwerveModuleState

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleState)
}
