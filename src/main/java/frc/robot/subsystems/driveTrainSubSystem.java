// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class driveTrainSubSystem extends SubsystemBase {
  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveTrainConstants.leftFrontCANID, CANSparkMaxLowLevel.MotorType.kBrushless);  // SPARK MAX for left front NEO motor.
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveTrainConstants.leftBackCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for left Back NEO motor.
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveTrainConstants.rightFrontCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for right Front NEO motor.
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveTrainConstants.rightBackCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for right Back NEO Motor


  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder righEncoder = rightFrontMotor.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  /** Creates a new ExampleSubsystem. */
  public driveTrainSubSystem() {
    // restore factory defaults for all motors
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults(); 
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    

    // set the values of the encoders to zero when robot starts
    leftEncoder.setPosition(0);
    righEncoder.setPosition(0);

    // make motors follow each other
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    rightControllerGroup.setInverted(false);
    leftControllerGroup.setInverted(true); // based on last year's code
  }
//   public void drive(double forward, double rotate, double fastMode, double slowMode){
//     double throttleFactor = 0.6;

//     if (fastMode > 0.2) {
//         IsFaster = true;
//     } else {
//         IsFaster = false;
//     }

//     if (slowMode > 0.2) {
//         IsSlower = true;
//     } else {
//         IsSlower = false;
//     }

//     if (IsFaster) {
//         throttleFactor = (0.40 * fastMode) + 0.6;
//     }

//     if (IsSlower) {
//         throttleFactor = (-0.2 * slowMode) + 0.6;
//     }

//     m_drive.arcadeDrive((forward * throttleFactor *-1), (rotate * throttleFactor*-1));
// }

  public void drive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
