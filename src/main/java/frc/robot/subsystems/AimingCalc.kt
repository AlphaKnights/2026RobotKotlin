package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import frc.robot.Constants.AimingConstants
import kotlin.math.*

object AimingCalc {
    fun canShoot(curPose: Pose2d): Double {
        // in meters
        var x: Double = -curPose.x
        var y: Double = curPose.y

        var distanceX: Double = abs(x-AimingConstants.HUB_X)
        var distanceY: Double = abs(y-AimingConstants.HUB_Y)
        var distanceTotal: Double = sqrt(distanceX.pow(2) + distanceY.pow(2))

        var distanceGood: Boolean = (AimingConstants.DISTANCE-AimingConstants.GOOD_DISTANCE_TOLERANCE <= distanceTotal
                && distanceTotal <= AimingConstants.DISTANCE+AimingConstants.GOOD_DISTANCE_TOLERANCE)
        var distanceMiddlingLow: Boolean = (AimingConstants.DISTANCE-AimingConstants.MIDDLING_DISTANCE_TOLERANCE <= distanceTotal
                && distanceTotal <= AimingConstants.DISTANCE &&
                !distanceGood)
        var distanceMiddlingHigh: Boolean = (AimingConstants.DISTANCE <= distanceTotal
                && distanceTotal <= AimingConstants.DISTANCE+AimingConstants.MIDDLING_DISTANCE_TOLERANCE &&
                !distanceGood)
        var distanceBadHigh: Boolean = (distanceTotal > AimingConstants.DISTANCE+AimingConstants.MIDDLING_DISTANCE_TOLERANCE)

        if (distanceBadHigh) {
            return 1.0
        } else if (distanceMiddlingHigh) {
            return 2.0
        } else if (distanceGood) {
            return 3.0
        } else if (distanceMiddlingLow) {
            return 4.0
        }
        return 5.0
    }
    fun getAimingAngleChange(curPose: Pose2d, vx: Double, vy: Double): Double {
        // in meters and radians
        var x: Double = -curPose.x
        var y: Double = curPose.y
        var angle: Double = curPose.rotation.radians

        var distanceX: Double = AimingConstants.HUB_X-x
        var distanceY: Double = AimingConstants.HUB_Y-y


        // (-dx/dt(-rx)+dy/dt(-ry))/(hx-rx)^2
        var termOne: Double = (vx-vy)/(distanceX.pow(2))

        // 1/(1+((hy-ry)/(hx-rx))^2)
        var termTwo: Double = 1/(1+(distanceY/distanceX).pow(2))

        // atan2(hy-ry,hx-rx)-angle
        var termThree: Double = atan2(distanceY, distanceX)-angle
        var angularDistance = 1.0
        if (abs(termThree) <= AimingConstants.SLOW_DISTANCE) {
            angularDistance = max(AimingConstants.MIN_SPEED, abs(termThree)/AimingConstants.SLOW_DISTANCE)
        }
        angularDistance = sqrt(angularDistance)

        return angularDistance*AimingConstants.MAX_SPEED+sign(x)*termOne*termTwo
    }
}