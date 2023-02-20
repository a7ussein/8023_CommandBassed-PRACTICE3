// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.driveTrainConstants;
import frc.robot.commands.driveTrainCommand;
import frc.robot.subsystems.driveTrainSubSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final driveTrainSubSystem driveTrainSubSystem = new driveTrainSubSystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final driveTrainCommand driveTrainCommand = new driveTrainCommand(driveTrainSubSystem);
  //CONTROLLERS 
  public static XboxController driveController = new XboxController(0); // Driving controller is supposed to be on port 0
  public static XboxController intakeController = new XboxController(1); // Intake controller is supposed to be on port 1 


    SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveTrainSubSystem.setDefaultCommand(driveTrainCommand);

    chooser.addOption("Deposit Cupe and Drive out", loadPathPlannerToRamseteCommand("E:/Robot_Code/src/main/deploy/deploy/pathplanner/DepostCubeAndExit.wpilib.json", true));
    chooser.addOption("Deposit And Exit Over Charging Station", loadPathPlannerToRamseteCommand("E:/Robot_Code/src/main/deploy/deploy/pathplanner/generatedJSON/DepositCubeAndExitOverChargningStation.wpilib.json", true));
    chooser.addOption("Go Pick An Object Up", loadPathPlannerToRamseteCommand("E:/Robot_Code/src/main/deploy/deploy/pathplanner/generatedJSON/GoPickAnObjectUp.wpilib.json", true));

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  public Command loadPathPlannerToRamseteCommand(String filename, boolean resetOdometry){
    Trajectory trajectory;
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to Open Trajectory " + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveTrainSubSystem::getPose,
        new RamseteController(driveTrainConstants.kRamseteB, driveTrainConstants.kRamseteZeta), 
        new SimpleMotorFeedforward(driveTrainConstants.ksVolts,driveTrainConstants.kvVoltSecondsPerMeter, 
        driveTrainConstants.kaVoltSecondsSquaredPerMeter), driveTrainConstants.kDriveKinematics, driveTrainSubSystem::getWheelSpeeds,
        new PIDController(driveTrainConstants.kPDriveVel, 0, 0), new PIDController(driveTrainConstants.kPDriveVel, 0, 0), 
        driveTrainSubSystem::tankDriveVolts, driveTrainSubSystem);

      if(resetOdometry){
        return new SequentialCommandGroup(new InstantCommand(()->driveTrainSubSystem.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
      }else{
        return ramseteCommand;
      }
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
  public driveTrainSubSystem getDriveTrainSubSystem(){
    return driveTrainSubSystem;
  }
}
