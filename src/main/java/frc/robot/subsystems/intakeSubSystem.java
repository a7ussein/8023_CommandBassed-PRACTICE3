package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.Constants;

public class intakeSubSystem {
    VictorSP intakeMotor1 = new VictorSP(Constants.intakeConstants.intakeMotor1);
    VictorSP intakeMotor2 = new VictorSP(Constants.intakeConstants.intakeMotor2);

    MotorControllerGroup intakeMotors = new MotorControllerGroup(intakeMotor1, intakeMotor2);
}
