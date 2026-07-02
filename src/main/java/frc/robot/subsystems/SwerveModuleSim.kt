package frc.robot.subsystems

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.Constants.DriveConstants

class SwerveModuleSim : SwerveModule {
    

    init {

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun getPosition(): SwerveModulePosition {
        TODO("Not yet implemented")
    }

    override fun getState(): SwerveModuleState {
        TODO("Not yet implemented")
    }

    override fun setDesiredState(desiredState: SwerveModuleState) {
        TODO("Not yet implemented")
    }

    companion object{
        private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
        private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    }
    }
}
