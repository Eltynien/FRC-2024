// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor1 = new CANSparkMax(ArmConstants.kArmMotorID1, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax motor2 = new CANSparkMax(ArmConstants.kArmMotorID2, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder;
  private double angle = ArmConstants.minAngle;
  private final PIDController pid = new PIDController(ArmConstants.Kp, ArmConstants.Ki, ArmConstants.Kd);

  /** Creates a new Arm. */
  public Arm() {
    encoder = motor1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  }

  public double getAngle(){
    return encoder.getPosition() * ArmConstants.encoderToAngle;
  }

  public void setAngle(double angle){
    this.angle = angle;
    if (angle < ArmConstants.minAngle){
      angle = ArmConstants.minAngle;
    }
    if (angle > ArmConstants.maxAngle){
      angle = ArmConstants.maxAngle;
    }
  }

  public void setMotor(double voltage){
    //motor1.setVoltage(voltage);
    //motor2.setVoltage(-voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setMotor(pid.calculate(getAngle(), angle));
    SmartDashboard.putNumber("Arm angle", getAngle());
  }
}
