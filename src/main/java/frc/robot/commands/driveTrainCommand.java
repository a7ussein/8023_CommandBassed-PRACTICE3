// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.driveTrainSubSystem;
import edu.wpi.first.wpilibj.Timer;
/*  import edu.wpi.first.wpilibj.Joystick; we are not using a joystick any more*/
import edu.wpi.first.wpilibj2.command.CommandBase;



public class driveTrainCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final driveTrainSubSystem driveTrainSubSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public driveTrainCommand(driveTrainSubSystem driveTrainSubSystem) {
    this.driveTrainSubSystem = driveTrainSubSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubSystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting drive train command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  private final double kDriveTick2Feet = 1.0/4096*6*Math.PI/12;
  final double kP = 0.5; // needs to be calculated, MAKE SURE YOU DON'T HAVE THE ROBOT ON THE GROUND UNTIL YOU FIGURE IT OUT!
  final double kI = 0; // needs to be calculated, MAKE SURE YOU DON'T HAVE THE ROBOT ON THE GROUND UNTIL YOU FIGURE IT OUT!
  final double kD = 0; // needs to be calculated, MAKE SURE YOU DON'T HAVE THE ROBOT ON THE GROUND UNTIL YOU FIGURE IT OUT!
  final double iLimit =  1;

  double setPoint = 0;
  double errorSum = 0;
  double lastTimeStamp = 0;
  double lastError = 0;

  @Override
  public void execute() {
    driveTrainSubSystem.resetEncoders();
    // double forwardSpeed = RobotContainer.driveController.getRawAxis(1);
    // double turningSpeed = RobotContainer.driveController.getRawAxis(4);
    // driveTrainSubSystem.drive(forwardSpeed, turningSpeed);
//Working Auto Code
    double outputSPeed = 0;
  if(RobotContainer.driveController.getXButton()){
    double angle = driveTrainSubSystem.m_IMU.getYComplementaryAngle();
      outputSPeed = 0;
      if(angle > 2){
        outputSPeed = -.05;
      }
      if(angle < -2){
        outputSPeed = .05;
      }
  }

  driveTrainSubSystem.leftControllerGroup.set(-outputSPeed);
  driveTrainSubSystem.rightControllerGroup.set(-outputSPeed);

  // PID (Proportional Integral Derivative) controlled Auto
  // Benefits: Slows down when the robot is near the setpoint
  // Stops Really accuritly on the setpoint.
  // Completes the movement very fast
  // Equation is Motor Output = kP * error
  // kP is a fixed number that is different from one robot to another
  // error is the distance between the setPoint and where the robot is located
  // if(RobotContainer.driveController.getXButton()){
  //   setPoint = 10;
  // }else if (RobotContainer.driveController.getBButton()){
  //   setPoint = 0;
  // }
  //  // get sensor position and convert it into feet
  //  double sensorPosition = driveTrainSubSystem.getLeftEncoderPosition() * kDriveTick2Feet;

  //  // calculations 
  //  double error = setPoint - sensorPosition;
  //  double dt = Timer.getFPGATimestamp() - lastTimeStamp;
   
  //  if(Math.abs(error) < iLimit){
  //    errorSum += errorSum * dt;
  //  }

  //  double errorRate = (error - lastError) /dt;
  //  double outputSpeed = kP * error + kI * errorSum + kD * errorRate;
   // double outputSpeed = kP * error + KI * errorSum;

   // output to motors 


   // update last - variables 
  //  lastTimeStamp = Timer.getFPGATimestamp();
  //  lastError = error;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
