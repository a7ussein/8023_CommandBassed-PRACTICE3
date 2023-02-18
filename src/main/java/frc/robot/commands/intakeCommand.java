package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intakeSubSystem;

public class intakeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final intakeSubSystem intakeSubSystem;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public intakeCommand(intakeSubSystem intakeSubSystem) {
      this.intakeSubSystem = intakeSubSystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intakeSubSystem);
    }
  
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Starting intake command");
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubSystem.startRaisingMotor(RobotContainer.intakeController.getRawAxis(1));
        intakeSubSystem.startRollerMotor(RobotContainer.intakeController.getRawAxis(3));
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
