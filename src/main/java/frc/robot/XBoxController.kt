package frc.robot

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

class XBoxController : XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) {
    fun x(): Double {
        return (-applyDeadband(
            getRawAxis(1),      //right y
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * speedScale()
                )
    }

    fun y(): Double {
        return (-applyDeadband(
            getRawAxis(0),    //right x
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * speedScale()
                )
    }

    fun rot(): Double {
        return (-applyDeadband(
            getRawAxis(4),   //left x
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * speedScale()
                )
    }
    fun speedScale(): Double{
        return ((-getRightTriggerAxis()+1))
    }


    fun heading() : Trigger {
        return Trigger { getYButton() }
    }
    fun alignL() : Trigger {
        return Trigger { getXButton()}
    }
    fun alignR() : Trigger {
        return Trigger { getBButton()}
    }





}
