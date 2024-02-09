// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetArmAngle extends Command {
  private double angle;
  private Arm arm;
  private final PIDController pid = new PIDController(ArmConstants.Kp, ArmConstants.Ki, ArmConstants.Kd);

  /** Creates a new SetArmAngle. */
  public SetArmAngle(int angle, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.arm = arm;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setMotor(pid.calculate(arm.getAngle(), angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
