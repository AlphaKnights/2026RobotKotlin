/*
 * (C) 2025 Galvaknights
 */
package frc.robot

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

class XBoxController : XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) {
    private var lerpX = 0.0
    private var lerpY = 0.0
    private var lerpRot = 0.0

    fun x(): Double {
        lerpX =
            lerp(
                -applyDeadband(
                    // right y
                    getRawAxis(1),
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpX,
            )
        return lerpX
    }

    fun y(): Double {
        lerpY =
            lerp(
                -applyDeadband(
                    // right x
                    getRawAxis(0),
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpY,
            )
        return lerpY
    }

    fun rot(): Double {
        lerpRot =
            lerp(
                -applyDeadband(
                    // left x
                    getRawAxis(4),
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpRot,
            )
        return lerpRot
    }

    fun speedScale(): Double = ((-getRightTriggerAxis() + 1))

    fun heading(): Trigger = Trigger { getYButton() }

    fun alignL(): Trigger = Trigger { getXButton() }

    fun alignR(): Trigger = Trigger { getBButton() }

    fun autoAim(): Trigger = Trigger { getAButton() }

    fun lerp(
        ref: Double,
        start: Double,
    ): Double {
        if (ref > start) {
            return if (start + Constants.OperatorConstants.LERP_VAL > ref) {
                ref
            } else {
                start + Constants.OperatorConstants.LERP_VAL
            }
        } else {
            return if (start - Constants.OperatorConstants.LERP_VAL < ref) {
                ref
            } else {
                start - Constants.OperatorConstants.LERP_VAL
            }
        }
    }
}
