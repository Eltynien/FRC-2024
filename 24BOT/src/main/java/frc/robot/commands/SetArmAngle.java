// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LimeLight;

public class SetArmAngle extends Command {
  private double angle;
  private Arm arm;

  /** Creates a new SetArmAngle. */
  public SetArmAngle(Arm arm, double angle) {
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  public SetArmAngle(Arm arm, LimeLight limelight) {
    if (limelight.isTargetAvailable()){
      angle = (85 + limelight.getTargetOffsetY() - LimelightConstants.limelightMountAngleDegrees) % 90;
    }
    else {
      angle = ArmConstants.shootingAngle;
    }
    this.arm = arm;

    addRequirements(arm, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(arm.getAngle() - angle) < 5);
  }
}
