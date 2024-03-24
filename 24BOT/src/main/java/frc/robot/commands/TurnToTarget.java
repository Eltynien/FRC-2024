// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToTarget extends Command {
  private final LimeLight limelight;
  private final SwerveSubsystem swerve;
  private final PIDController pid = new PIDController(DriveTrainConstants.kPRotation, DriveTrainConstants.kIRotation, DriveTrainConstants.kDRotation);

  /** Creates a new TurnToTarget. */
  public TurnToTarget(LimeLight limelight, SwerveSubsystem swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get the current speeds and only change the rotation component using pid on the amount of offset from target
    ChassisSpeeds speeds = swerve.getChassisSpeeds();
    speeds.omegaRadiansPerSecond = pid.calculate(limelight.getTargetOffsetX(), 0);
    swerve.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setChassisSpeeds(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limelight.getTargetOffsetX() < 0.05){
      return true;
    }
    return false;
  }
}
