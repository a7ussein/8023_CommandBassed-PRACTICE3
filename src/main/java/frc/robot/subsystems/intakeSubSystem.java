package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSubSystem extends SubsystemBase{
    WPI_VictorSPX rollerMotor = new WPI_VictorSPX(Constants.intakeConstants.intakeMotor1);
    WPI_VictorSPX raisingMotor = new WPI_VictorSPX(Constants.intakeConstants.intakeMotor2);


    // Roller Motor functions
    public void startRollerMotor(double speed){
        rollerMotor.set(speed);
    }   
    // Rasing Motor functions
    public void startRaisingMotor(double speed){
        raisingMotor.set(speed);
    }
}
