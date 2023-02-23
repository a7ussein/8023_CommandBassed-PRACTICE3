// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Drive Train Constants 
  public static final class driveTrainConstants{
    public static final int leftFrontCANID = 3;
    public static final int leftBackCANID = 4;
    public static final int rightFrontCANID = 2; // inverted
    public static final int rightBackCANID = 1; // inverted

  //Replace the below values with numbers from SysId
    public static final double ksVolts = 0.20322; // Replace 
    public static final double kvVoltSecondsPerMeter = 3.2976; // Replace
    public static final double kaVoltSecondsSquaredPerMeter = 0.67542; // Replace
    public static final double kPDriveVel = 0.5; // Replace

    public static final double kTrackWidthMeters = Units.inchesToMeters(14); // according to Alex
    public static final double kDistanceFromMotorToFrontOfTheChasie = Units.inchesToMeters(14.5); // bassed on missurement
  //-------
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
      
    public static final double kMaxSpeedMetersPerSecond = 3; // Leave it as it is
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; // Leave it as it is

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2; // Leave it as it is 
    public static final double kRamseteZeta = 0.7; // Leave it as it is

    public static final double kGearRatio = 12.6;
    public static final double kWheelRadiusInches = 3;

    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));

  }

  // Intake Constants 
  public static final class intakeConstants{
    public static final int intakeMotor1 = 5;
    public static final int intakeMotor2 = 6;
  }
  // Operator Constants 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kIntakeControllerPort = 1;
  }
}
