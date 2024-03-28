// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
    DriveTrainConstants.kFrontLeftDriveMotorID,
    DriveTrainConstants.kFrontLeftTurnMotorID,
    DriveTrainConstants.kFrontLeftDriveMotorReversed,
    DriveTrainConstants.kFrontLeftTurnMotorReversed,
    DriveTrainConstants.kFrontLeftAbsoluteEncoderOffset,
    DriveTrainConstants.kFrontLeftAbsoluteEncoderReversed
  );

  private final SwerveModule frontRight = new SwerveModule(
    DriveTrainConstants.kFrontRightDriveMotorID,
    DriveTrainConstants.kFrontRightTurnMotorID,
    DriveTrainConstants.kFrontRightDriveMotorReversed,
    DriveTrainConstants.kFrontRightTurnMotorReversed,
    DriveTrainConstants.kFrontRightAbsoluteEncoderOffset,
    DriveTrainConstants.kFrontRightAbsoluteEncoderReversed
  );

  private final SwerveModule backLeft = new SwerveModule(
    DriveTrainConstants.kBackLeftDriveMotorID,
    DriveTrainConstants.kBackLeftTurnMotorID,
    DriveTrainConstants.kBackLeftDriveMotorReversed,
    DriveTrainConstants.kBackLeftTurnMotorReversed,
    DriveTrainConstants.kBackLeftAbsoluteEncoderOffset,
    DriveTrainConstants.kBackLeftAbsoluteEncoderReversed
  );

  private final SwerveModule backRight = new SwerveModule(
    DriveTrainConstants.kBackRightDriveMotorID,
    DriveTrainConstants.kBackRightTurnMotorID,
    DriveTrainConstants.kBackRightDriveMotorReversed,
    DriveTrainConstants.kBackRightTurnMotorReversed,
    DriveTrainConstants.kBackRightAbsoluteEncoderOffset,
    DriveTrainConstants.kBackRightAbsoluteEncoderReversed
  );
  
  private final Field2d field = new Field2d();

  private final AHRS gyroscope = new AHRS(SPI.Port.kMXP); 

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
    DriveTrainConstants.kDriveKinematics, 
    gyroscope.getRotation2d(),
    new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    }
  );
  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    SmartDashboard.putData("Field", field);

    gyroscope.reset();

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(DriveTrainConstants.kPTranslation, DriveTrainConstants.kITranslation, DriveTrainConstants.kDTranslation), // Translation PID constants
              new PIDConstants(DriveTrainConstants.kPRotation, DriveTrainConstants.kIRotation, DriveTrainConstants.kDRotation), // Rotation PID constants
              DriveTrainConstants.kPhysicalMaxVelocity, // Max module speed, in m/s
              Math.sqrt(Math.pow(DriveTrainConstants.kTrackWidth, 2) + Math.pow(DriveTrainConstants.kWheelBase, 2)), 
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, // return if need to flip path
      this // Reference to this subsystem to set requirements
    );

  }

  public double getHeading() {
    return (Math.IEEEremainder(-gyroscope.getAngle(), 360));
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void resetGyroscope(){
    gyroscope.reset();
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveTrainConstants.kPhysicalMaxVelocity);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds () {
    return DriveTrainConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(),
      backLeft.getState(), backRight.getState()
    });
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setModuleStates(DriveTrainConstants.kDriveKinematics.toSwerveModuleStates(speeds));
  }

  
  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry (Pose2d pose) {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPositionOdometry(), frontRight.getPositionOdometry(),
      backLeft.getPositionOdometry(), backRight.getPositionOdometry()
    }, pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPositionOdometry(), frontRight.getPositionOdometry(),
      backLeft.getPositionOdometry(), backRight.getPositionOdometry()
    });


    SmartDashboard.putNumber("Gyro Angle", getHeading());

    // Do this in either robot periodic or subsystem periodic
    field.setRobotPose(odometer.getPoseMeters());

    frontLeft.showDebugInfo();
    frontRight.showDebugInfo();
    backLeft.showDebugInfo();
    backRight.showDebugInfo();
  }
}
