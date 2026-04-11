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
                    getRawAxis(1), // right y
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpX,
            )
        return lerpX
    }

    fun y(): Double {
        lerpY = lerp(-applyDeadband(
            getRawAxis(0),    //right x
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * speedScale(),
            lerpY
                )
        return lerpY
    }

    fun rot(): Double {
        lerpRot = lerp(-applyDeadband(
            getRawAxis(4),   //left x
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * speedScale(),
            lerpRot
                )
        return lerpRot
    }
    fun speedScale(): Double{
        return ((-getRightTriggerAxis() +1))
    }

    fun deliveryScale(): Double {
        return getLeftTriggerAxis()
    }

    fun heading() : Trigger {
        return Trigger { yButton }
    }
    fun alignL() : Trigger {
        return Trigger { xButton }
    }
   // fun alignR() : Trigger {
     //   return Trigger { bButton }
   // }
    fun autoAim() : Trigger {
        return Trigger { aButton }
    }

    fun lerp(ref: Double, start: Double) : Double {
        if (ref > start) {
            return if (start + Constants.OperatorConstants.LERP_VAL > ref) ref
            else start + Constants.OperatorConstants.LERP_VAL
        } else {
            return if (start - Constants.OperatorConstants.LERP_VAL < ref) ref
            else start - Constants.OperatorConstants.LERP_VAL
        }

    }

    fun resetOdometry(): Trigger {
        return Trigger { rightStickButton }
    }

    fun driveToArc(): Trigger {
        return Trigger { startButton }
    }

//    fun slideLeft(): Trigger {
//        return Trigger { leftBumperButton }
//    }
//
//    fun slideRight(): Trigger {
//        return Trigger { rightBumperButton }
//    }

    fun north(): Trigger {
        return Trigger { startButton } // Select Button
    }

    fun XLock(): Trigger {
        return Trigger { leftBumperButton }
    }

    fun altDelivery(): Trigger {
        return Trigger { aButton }
    }

    fun altIntake(): Trigger {
        return Trigger { rightBumperButton }
    }

    fun altIndexer(): Trigger {
        return Trigger { bButton }
    }


}
