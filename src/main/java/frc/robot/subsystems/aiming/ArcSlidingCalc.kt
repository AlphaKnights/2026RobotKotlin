package frc.robot.subsystems.aiming

import frc.robot.Constants.AimingConstants
import kotlin.math.sin

object ArcSlidingCalc {
    fun getXChange(angleChange: Double): Double {
        return AimingConstants.DISTANCE * sin(angleChange)
    }
    fun getYChange(angleChange: Double): Double {
        return AimingConstants.DISTANCE * angleChange
    }
}