// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static class OIConstants {
    public static int kDriverControllerPort1 = 0;
    public static int kDriverControllerPort2 = 1;
    public static double kDeadband = 0.05;
  }
  
  public static class LimelightConstants {
    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = 28.0; 

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightMeters = Units.inchesToMeters(14.0); 

    // distance from the target to the floor
    public static double goalHeightMeters= Units.inchesToMeters(78.2); 

    // turn constant
    public static double kP = .006;
  }

  public static class DriveTrainConstants {

    // swerve module intialization information
    public static int kBackRightDriveMotorID = 1;
    public static int kBackRightTurnMotorID = 5;
    public static boolean kBackRightAbsoluteEncoderReversed = true;
    public static double kBackRightAbsoluteEncoderOffset = -995;
    public static boolean kBackRightDriveMotorReversed = false;
    public static boolean kBackRightTurnMotorReversed = true;

    public static int kBackLeftDriveMotorID = 2;
    public static int kBackLeftTurnMotorID = 6;
    public static boolean kBackLeftAbsoluteEncoderReversed = true;
    public static double kBackLeftAbsoluteEncoderOffset = -432;
    public static boolean kBackLeftDriveMotorReversed = true;
    public static boolean kBackLeftTurnMotorReversed = true;

    public static int kFrontRightDriveMotorID = 3;
    public static int kFrontRightTurnMotorID = 7;
    public static boolean kFrontRightAbsoluteEncoderReversed = true;
    public static double kFrontRightAbsoluteEncoderOffset = -2233;
    public static boolean kFrontRightDriveMotorReversed = false; 
    public static boolean kFrontRightTurnMotorReversed = false;

    public static int kFrontLeftDriveMotorID = 4;
    public static int kFrontLeftTurnMotorID = 8;
    public static boolean kFrontLeftAbsoluteEncoderReversed = true;
    public static double kFrontLeftAbsoluteEncoderOffset =  635;
    public static boolean kFrontLeftDriveMotorReversed = true;
    public static boolean kFrontLeftTurnMotorReversed = true;

    // physical info
    public static double kTrackWidth = Units.inchesToMeters(19.234);
    public static double kWheelBase = Units.inchesToMeters(19.234);
    public static double kWheelRadius = Units.inchesToMeters(2);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static double kTurnEncoderResolution = 4096.0;

    // conversion factors
    public static double kTurnEncoderToRad = 2.0 * Math.PI /kTurnEncoderResolution; 
    public static double kDriveEncoderToMeters = Math.PI * 2 * kWheelRadius * 0.142857;
  
    // pid for the swerve modules
    public static double kPTurning = 0.5;
    public static double kITurning = 0;
    public static double kDTurning = 0.01;

    public static double kSDrive = 0.05;
    public static double kVDrive = 0.14;
    public static double kPDrive = 0.10;
    public static double kIDrive = 0.0;
    public static double kDDrive = 0;

    // pid for the trajectory following

    public static final double kPTranslation = 0.8; 
    public static final double kITranslation = 0;
    public static final double kDTranslation = 0;

    public static final double kPRotation = 0.07; 
    public static final double kIRotation = 0;
    public static final double kDRotation = 0;

    // maximums and limits
    public static final double kPhysicalMaxVelocity = 5;
    public static final double kPhysicalMaxAngularVelocity = 3 * Math.PI;

    public static final double kDriveMaxAcceleration = 10;
    public static final double kDriveMaxAngularAcceleration = 3 * Math.PI; 
    public static final double kDesiredMaxSpeed = 5; 
  }
  
  public static class ArmConstants {
    public static int kArmMotorID1 = 9;
    public static int kArmMotorID2 = 10;
    public static double Kp = 0.26;
    public static double Ki = 0.001;
    public static double Kd = 0.0007;
    public static int armEncoderPort1 = 3;
    public static int armEncoderPort2 = 2;
    public static double encoderToAngle = 360/8192;
    public static double minAngle = 5;
    public static double maxAngle = 90;

    public static double kIntakeAngle = 5;
    public static double shootingAngle = 55;
  }

    public static class IntakeConstants {
    public static int kIntakeMotorPort1 = 11;
    public static int kIntakeMotorPort2 = 12;
    public static boolean kMotor1Inverted = false;
    public static boolean kMotor2Inverted = false;
  }

  public static class ShooterConstants {
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    
    public static int kShooterMotorOneId = 13;
    public static int kShooterMotorTwoId = 14;
  }

}
