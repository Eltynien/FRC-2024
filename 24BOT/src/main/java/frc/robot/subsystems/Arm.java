// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor1 = new CANSparkMax(ArmConstants.kArmMotorID1, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax motor2 = new CANSparkMax(ArmConstants.kArmMotorID2, CANSparkLowLevel.MotorType.kBrushed);
  private final Encoder encoder = new Encoder(ArmConstants.armEncoderPort1, ArmConstants.armEncoderPort2);

  /** Creates a new Arm. */
  public Arm() {}

  public double getAngle(){
    return encoder.getDistance() * ArmConstants.encoderToAngle;
  }

  public void setMotor(double voltage){
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm angle", getAngle());
  }
}
