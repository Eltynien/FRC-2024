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
    public static int kDriverControllerPort = 0;
    public static double kDeadband = 0.05;
  }

  public static class DriveTrainConstants {

    // swerve module intialization information
    public static int kFrontLeftDriveMotorID = 1;
    public static int kFrontLeftTurnMotorID = 5;
    public static boolean kFrontLeftAbsoluteEncoderReversed = false;
    public static double kFrontLeftAbsoluteEncoderOffset = 962;
    public static boolean kFrontLeftDriveMotorReversed = true;
    public static boolean kFrontLeftTurnMotorReversed = false;

    public static int kFrontRightDriveMotorID = 2;
    public static int kFrontRightTurnMotorID = 6;
    public static boolean kFrontRightAbsoluteEncoderReversed = false;
    public static double kFrontRightAbsoluteEncoderOffset = 0;
    public static boolean kFrontRightDriveMotorReversed = true;
    public static boolean kFrontRightTurnMotorReversed = false;

    public static int kBackLeftDriveMotorID = 3;
    public static int kBackLeftTurnMotorID = 7;
    public static boolean kBackLeftAbsoluteEncoderReversed = true;
    public static double kBackLeftAbsoluteEncoderOffset = 2219;
    public static boolean kBackLeftDriveMotorReversed = false; 
    public static boolean kBackLeftTurnMotorReversed = true;

    public static int kBackRightDriveMotorID = 4;
    public static int kBackRightTurnMotorID = 8;
    public static boolean kBackRightAbsoluteEncoderReversed = false; // absolute encoder does not work
    public static double kBackRightAbsoluteEncoderOffset = 618; // absolute encoder does not seem to work
    public static boolean kBackRightDriveMotorReversed = true;
    public static boolean kBackRightTurnMotorReversed = false;

    // physical info
    public static double kTrackWidth = Units.inchesToMeters(22.5);
    public static double kWheelBase = Units.inchesToMeters(22.5);
    public static double kDriveGearRatio = 0.266667;
    public static double kTurnGearRatio = 0.01786;
    public static double kWheelRadius = Units.inchesToMeters(2);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase/2, -kTrackWidth/2),
      new Translation2d(kWheelBase/2, kTrackWidth/2),
      new Translation2d(-kWheelBase/2, -kTrackWidth/2),
      new Translation2d(-kWheelBase/2, kTrackWidth/2)
    );

    public static double kEncoderResolution= 4096.0;

    // conversion factors
    public static double kTurnEncoderToRad= 2 * Math.PI /4096; //* kTurnGearRatio;
    public static double kDriveEncoderToMeters = 1 / 4096 * Math.PI * 2 * kWheelRadius * kDriveGearRatio;
  
    // pid for the swerve modules
    public static double kPTurning = 0.5;
    public static double kITurning = 0;
    public static double kDTurning = 0;

    public static double kPDrive = 1;
    public static double kIDrive = 0;
    public static double kDDrive = 0;

    // pid for the trajectory following
    public static final double kPTranslation = 1.5;
    public static final double kITranslation = 0;
    public static final double kDTranslation = 0;

    public static final double kPRotation = 3;
    public static final double kIRotation = 0;
    public static final double kDRotation = 0;

    // maximums and limits
    public static final double kPhysicalMaxVelocity = 5;
    public static final double kPhysicalMaxAngularVelocity = 2 * 2 * Math.PI;

    public static final double kDriveMaxAcceleration = 5;
    public static final double kDriveMaxAngularAcceleration = kPhysicalMaxAngularVelocity/4;
    public static final double kDesiredMaxSpeed = kPhysicalMaxVelocity / 4;
  }
}
