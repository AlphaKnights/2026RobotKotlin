package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import frc.robot.Constants.AimingConstants
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

object AimingCalc {
    fun canShoot(curPose: Pose3d): Int {
        // in meters
        var x: Double = -curPose.x
        var y: Double = curPose.z

        var distanceX: Double = abs(x-AimingConstants.HUB_X)
        var distanceY: Double = abs(y-AimingConstants.HUB_Y)
        var distanceTotal: Double = sqrt(distanceX.pow(2) + distanceY.pow(2))

        var distanceGood: Boolean = (AimingConstants.DISTANCE-AimingConstants.GOOD_DISTANCE_TOLERANCE <= distanceTotal
                && distanceTotal <= AimingConstants.DISTANCE+AimingConstants.GOOD_DISTANCE_TOLERANCE)
        var distanceMiddling: Boolean = (AimingConstants.DISTANCE-AimingConstants.MIDDLING_DISTANCE_TOLERANCE <= distanceTotal
                && distanceTotal <= AimingConstants.DISTANCE+AimingConstants.MIDDLING_DISTANCE_TOLERANCE &&
                !distanceGood)

        if (distanceGood) {
            return 1
        } else if (distanceMiddling) {
            return 2
        } else {
            return 3
        }
    }
    fun getAimingAngleChange(curPose: Pose3d, vx: Double, vy: Double): Double {
        // in meters and radians
        var x: Double = -curPose.x
        var y: Double = curPose.z

        var distanceX: Double = AimingConstants.HUB_X-x
        var distanceY: Double = AimingConstants.HUB_Y-y


        // (-dx/dt(-rx)+dy/dt(-ry))/(hx-rx)^2
        // (rx-px-(ry-py))/(hx-rx)^2
        var termOne: Double = (vx-vy)/(distanceX.pow(2))

        // 1/(1+((hy-ry)/(hx-rx))^2)
        var termTwo: Double = 1/(1+(distanceY/distanceX).pow(2))

        return sign(distanceX)*termOne*termTwo
    }
}