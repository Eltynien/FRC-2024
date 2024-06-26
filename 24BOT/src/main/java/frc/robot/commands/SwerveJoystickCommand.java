// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends Command {
  /** Creates a new SwerveJoystickCommand. */
  private final SwerveSubsystem swerveSubsystem;
  private final LimeLight limeLight;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, limelightFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem, LimeLight limeLight, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Double> limelightFunction) {
 // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.limeLight = limeLight;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.limelightFunction = limelightFunction;

    this.xLimiter = new SlewRateLimiter(DriveTrainConstants.kDriveMaxAcceleration);
    this.yLimiter = new SlewRateLimiter(DriveTrainConstants.kDriveMaxAcceleration);
    this.turningLimiter = new SlewRateLimiter(DriveTrainConstants.kDriveMaxAngularAcceleration);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get values
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    boolean fieldRelative = true;
  
    // apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;


    // rate limit
    xSpeed = xLimiter.calculate(xSpeed) * DriveTrainConstants.kDesiredMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * DriveTrainConstants.kDesiredMaxSpeed;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveTrainConstants.kDriveMaxAngularAcceleration;


     // limelight
    if (limelightFunction.get() > 0){
      turningSpeed = - limeLight.getTargetOffsetX() * LimelightConstants.kP * DriveTrainConstants.kDriveMaxAngularAcceleration;
    }
    SmartDashboard.putBoolean("Limelight Active", limelightFunction.get() > 0);

    // convert to chassis speeds
    ChassisSpeeds chassisSpeeds;
    
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    if (fieldRelative){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    swerveSubsystem.setChassisSpeeds(chassisSpeeds);    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
