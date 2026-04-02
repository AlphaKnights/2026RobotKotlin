package frc.robot.subsystems

import frc.robot.Constants.AimingConstants
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sinh

object ArcSlidingCalc {
    fun getXChange(angleChange: Double): Double {
        return AimingConstants.DISTANCE * sin(angleChange)
    }
    fun getYChange(angleChange: Double): Double {
        return AimingConstants.DISTANCE * cos(angleChange)
    }
}