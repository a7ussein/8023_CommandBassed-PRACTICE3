package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSubSystem extends SubsystemBase{
    VictorSP rollerMotor = new VictorSP(Constants.intakeConstants.intakeMotor1);
    VictorSP raisingMotor = new VictorSP(Constants.intakeConstants.intakeMotor2);


    // Roller Motor functions
    public void startRollerMotor(double speed){
        rollerMotor.set(speed);
    }   
    // Rasing Motor functions
    public void startRaisingMotor(double speed){
        raisingMotor.set(speed);
    }
}
