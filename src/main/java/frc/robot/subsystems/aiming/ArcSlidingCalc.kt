/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.aiming

import frc.robot.Constants.AimingConstants
import kotlin.math.sin

object ArcSlidingCalc {
    fun getXChange(angleChange: Double): Double = AimingConstants.DISTANCE * sin(angleChange)

    fun getYChange(angleChange: Double): Double = AimingConstants.DISTANCE * angleChange
}
