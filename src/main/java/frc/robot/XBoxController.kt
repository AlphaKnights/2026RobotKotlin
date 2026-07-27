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
                    getRawAxis(1),
                    // right y
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
                    getRawAxis(0),
                    // right x
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
                    getRawAxis(2),
                    // left x
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpRot,
            )
        return lerpRot
    }

    fun speedScale(): Double = ((-rightTriggerAxis + 1))

    fun deliveryScale(): Double = leftTriggerAxis

    fun heading(): Trigger = Trigger { yButton }

    fun alignL(): Trigger = Trigger { xButton }

    // fun alignR() : Trigger {
    //   return Trigger { bButton }
    // }
    fun autoAim(): Trigger = Trigger { aButton }

    fun lerp(
        ref: Double,
        start: Double,
    ): Double =
        if (ref > start) {
            if (start + Constants.OperatorConstants.LERP_VAL > ref) {
                ref
            } else {
                start + Constants.OperatorConstants.LERP_VAL
            }
        } else {
            if (start - Constants.OperatorConstants.LERP_VAL < ref) {
                ref
            } else {
                start - Constants.OperatorConstants.LERP_VAL
            }
        }

    fun resetOdometry(): Trigger = Trigger { rightStickButton }

    fun driveToArc(): Trigger = Trigger { getRawButton(12) }

    fun slideLeft(): Trigger = Trigger { leftBumperButton }

    fun slideRight(): Trigger = Trigger { rightBumperButton }

    fun north(): Trigger {
        return Trigger { getRawButton(11) } // Select Button
    }

    fun xLock(): Trigger = Trigger { leftBumperButton }

    fun altDelivery(): Trigger = Trigger { aButton }

    fun altIntake(): Trigger = Trigger { rightBumperButton }

    fun altIndexer(): Trigger = Trigger { bButton }
}
