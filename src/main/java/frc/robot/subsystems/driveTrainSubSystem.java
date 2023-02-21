// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.driveTrainConstants;

public class driveTrainSubSystem extends SubsystemBase {

  private DifferentialDriveOdometry m_Odometry; // based on some online code that can be found in the NotesRelatedToTheIMU.txt file

  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveTrainConstants.leftFrontCANID, CANSparkMaxLowLevel.MotorType.kBrushless);  // SPARK MAX for left front NEO motor.
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveTrainConstants.leftBackCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for left Back NEO motor.
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveTrainConstants.rightFrontCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for right Front NEO motor.
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveTrainConstants.rightBackCANID, CANSparkMaxLowLevel.MotorType.kBrushless); // SPARK MAX for right Back NEO Motor


  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  // IMU code
  // According to https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/ADIS16470_IMU.html
  // The IMU library will perform a calibartion for the imu in its constructor
  public final static ADIS16470_IMU m_IMU = new ADIS16470_IMU();
  public final static ADIS16470_IMUSim m_IMUSim = new ADIS16470_IMUSim(m_IMU);

  // DriveTrain SubSystem
  public driveTrainSubSystem() {
    // restore factory defaults for all motors
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults(); 
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    

    // set the values of the encoders to zero when robot starts
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightEncoder.setPositionConversionFactor(driveTrainConstants.kLinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(driveTrainConstants.kLinearDistanceConversionFactor);
    rightEncoder.setVelocityConversionFactor(driveTrainConstants.kLinearDistanceConversionFactor/60);
    leftEncoder.setVelocityConversionFactor(driveTrainConstants.kLinearDistanceConversionFactor/60);
    

    // make motors follow each other
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    rightControllerGroup.setInverted(false);
    leftControllerGroup.setInverted(true); // based on last year's code
    // it is a good practice to reset everything in the drive train subsystem.
    m_IMU.reset();
    m_IMU.calibrate();
    resetEncoders();

    m_Odometry = new DifferentialDriveOdometry(new Rotation2d(Units.degreesToRadians(getHeading())), leftEncoder.getPosition(), rightEncoder.getPosition());
    m_Odometry.resetPosition(new Rotation2d(Units.degreesToRadians(getHeading())), getLeftEncoderPosition(), getRightEncoderPosition(), new Pose2d());
  }

// _____________________________________________________________M E T H O D S________________________________________________________
// Driving Methods
  // Drive Method:
  public void drive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  // Votlts Controlling Method
  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftControllerGroup.setVoltage(leftVolts);
    rightControllerGroup.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  // Motors Maximum speed method
  public void setMaxOutput(double maxOutput){
    differentialDrive.setMaxOutput(maxOutput);
  }

  // BreakMode method
  public void setBreakMode(){
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
  }

  // CoastMode method
  public void setCoastMode(){
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);
  }
//_________________________________________________
// Encoder Methods:
  // Encoder reset:
public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
// Wheel Speed Method 
public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
}

// Average encoder distance Method
public double getAverageEncoderDistance(){
  return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0) ;
}

// Position of the right encoder after the conversion factor is applied Method --- had to negate it cuz it was outputing results in negative nums
public double getRightEncoderPosition(){
  return rightEncoder.getPosition() + Constants.driveTrainConstants.kDistanceFromMotorToFrontOfTheChasie;
}


// Position of the left encoder after the conversion factor is applied Method
public double getLeftEncoderPosition(){
  return leftEncoder.getPosition() + Constants.driveTrainConstants.kDistanceFromMotorToFrontOfTheChasie;
}


// Velocity of the right encoder after the conversion factor is applied Method
public double getRightEncoderVelocity(){
  return (rightEncoder.getVelocity());
}

// Velocity of the left encoder after the conversion factor is applied  Method
public double getLeftEncoderVelocity(){
  return leftEncoder.getVelocity();
}

// Methods to return the Encoders:
public RelativeEncoder getLefEncoder(){
  return leftEncoder;
}

public RelativeEncoder getRightEncoder(){
  return rightEncoder;

}

//____________________________
// IMU related Methods:
  // Odometry Reset
  public void resetOdometry(Pose2d poss){
    resetEncoders();
    m_Odometry.resetPosition(new Rotation2d(Units.degreesToRadians(getHeading())), getLeftEncoderPosition(), getRightEncoderPosition(), poss);
  }

  // Heading Reset
  public void zeroHeading(){
    m_IMU.calibrate();
    m_IMU.reset();
  }

  public ADIS16470_IMU getImu(){
    return getImu();
  }

 // Heading Output Method:
  public double getHeading() {
      return -m_IMU.getAngle();
  }

  // Turning Rate Output Method:
  public double getTurnRate(){
    return -m_IMU.getRate();
  }
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }

  // 
//_________________________________________________

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(new Rotation2d(Units.degreesToRadians(getHeading())), leftEncoder.getPosition(), rightEncoder.getPosition());

    // Put some values in the smartDashboard
    SmartDashboard.putNumber("Left encoder value in meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder value in meters", getRightEncoderPosition());
    SmartDashboard.putNumber("left Encoder Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("right Encoder velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("Imu heading", getHeading());
    SmartDashboard.putNumber("Imu Turn Rate", getTurnRate());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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

/*
last year's driving code  
public void drive(double forward, double rotate, double fastMode, double slowMode){
    double throttleFactor = 0.6;

    if (fastMode > 0.2) {
        IsFaster = true;
    } else {
        IsFaster = false;
    }

    if (slowMode > 0.2) {
        IsSlower = true;
    } else {
        IsSlower = false;
    }

    if (IsFaster) {
        throttleFactor = (0.40 * fastMode) + 0.6;
    }

    if (IsSlower) {
        throttleFactor = (-0.2 * slowMode) + 0.6;
    }

    m_drive.arcadeDrive((forward * throttleFactor *-1), (rotate * throttleFactor*-1));
}
*/

}
