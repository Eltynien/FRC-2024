// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final Spark motor = new Spark(ArmConstants.kArmMotorPort);
  private final Encoder encoder = new Encoder(ArmConstants.armEncoderArm1, ArmConstants.armEncoderArm2);
  /** Creates a new Arm. */
  public Arm() {}

  public double getAngle(){
    return encoder.getDistance() * ArmConstants.encoderToAngle;
  }

  public void setMotor(double voltage){
    motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
