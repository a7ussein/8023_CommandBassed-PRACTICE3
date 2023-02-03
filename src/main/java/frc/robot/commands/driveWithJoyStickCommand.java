// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveTrainSubSystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
gvfv c
/** An example command that uses an example subsystem. */
public class driveWithJoyStickCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final driveTrainSubSystem driveTrainSubSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public driveWithJoyStickCommand(driveTrainSubSystem driveTrainSubSystem) {
    this.driveTrainSubSystem = driveTrainSubSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubSystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Joystick drive command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = RobotContainer.joystick.getX();
    double turningSPeed = RobotContainer.joystick.getZ();
    driveTrainSubSystem.arcadeDrive(forwardSpeed, turningSPeed);
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
