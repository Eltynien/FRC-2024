// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor1 = new CANSparkMax(ArmConstants.kArmMotorID1, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax motor2 = new CANSparkMax(ArmConstants.kArmMotorID2, CANSparkLowLevel.MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  private double angle = ArmConstants.kIntakeAngle;
  private final PIDController pid = new PIDController(ArmConstants.Kp, ArmConstants.Ki, ArmConstants.Kd);

  /** Creates a new Arm. */
  public Arm() {
    encoder.reset();
    encoder.setDistancePerRotation(360);
    motor1.setIdleMode(IdleMode.kBrake);
    motor2.setIdleMode(IdleMode.kBrake);
  }

  public double getAngle(){
    return encoder.getDistance();
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
    motor1.setVoltage(voltage);
    motor2.setVoltage(-voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidValue = pid.calculate(getAngle(), angle);
    if (pidValue > 5.0){
      pidValue = 5.0;
    }
    if (pidValue < -5.0){
      pidValue = -5.0;
    }
    setMotor(pidValue);
    SmartDashboard.putNumber("Arm Angle", encoder.getDistance());
    }
}
