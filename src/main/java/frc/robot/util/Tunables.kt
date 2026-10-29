/*
 * (C) 2025 Galvaknights
 */
package frc.robot.util

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

/**
 * Values you can change from the dashboard while the robot runs, so tuning doesn't need a redeploy.
 *
 * Each one starts at the value from Constants and shows up under its key. Edit it live; once it
 * works, copy it into Constants.kt, because every reboot starts from the code value again.
 */
object Tunables {
    fun number(
        key: String,
        default: Double,
    ): Double {
        SmartDashboard.setDefaultNumber(key, default)
        return SmartDashboard.getNumber(key, default)
    }

    fun flag(
        key: String,
        default: Boolean,
    ): Boolean {
        SmartDashboard.setDefaultBoolean(key, default)
        return SmartDashboard.getBoolean(key, default)
    }
}
