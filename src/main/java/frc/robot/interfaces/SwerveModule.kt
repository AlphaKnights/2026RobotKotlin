/*
 * (C) 2025 Galvaknights
 */
package frc.robot.interfaces

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

interface SwerveModule {
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
