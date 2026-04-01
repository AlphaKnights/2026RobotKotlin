package frc.robot.subsystems

import frc.robot.Constants.AimingConstants
import kotlin.math.cos
import kotlin.math.sin

object ArcSlidingCalc {
    fun getXChange(angleChange: Double): Double {
        return AimingConstants.DISTANCE * cos(angleChange)
    }
    fun getYChange(angleChange: Double): Double {
        return AimingConstants.DISTANCE * sin(angleChange)
    }
}