package frc.robot

import edu.wpi.first.math.MathUtil.applyDeadband
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger

class JoystickController : Joystick(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) {
    fun x(): Double {
        return (-applyDeadband(
            getRawAxis(0),
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * (-getRawAxis(3) + 1) / 2
                )
    }

    fun y(): Double {
        return (-applyDeadband(
            getRawAxis(1),
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * (-getRawAxis(3) + 1) / 2
                )
    }

    fun rot(): Double {
        return (-applyDeadband(
            getRawAxis(4),
            Constants.OperatorConstants.DRIVE_DEADBAND
        ) * (-getRawAxis(3) + 1) / 2
                )
    }

    fun heading() : Trigger {
        return JoystickButton(this, Constants.OperatorConstants.RESET_HEADING_BUTTON)
    }
    fun alignL() : Trigger {
        return JoystickButton(this, Constants.OperatorConstants.ALIGN_LEFT_BUTTON)
    }
    fun alignR() : Trigger {
        return JoystickButton(this, Constants.OperatorConstants.ALIGN_RIGHT_BUTTON)
    }
}
