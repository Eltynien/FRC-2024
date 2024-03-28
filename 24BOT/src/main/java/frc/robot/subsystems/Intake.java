// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
  CANSparkMax sparkMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotorPort1, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax sparkMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotorPort2, CANSparkLowLevel.MotorType.kBrushless);
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

/** Creates a new Intake. */
  public Intake() {
    //sparkMotor1.setInverted(IntakeConstants.kMotor1Inverted);
    //sparkMotor2.setInverted(IntakeConstants.kMotor2Inverted);
  }

  public void setMotor(double voltage){
    sparkMotor1.setVoltage(-voltage);
    sparkMotor2.setVoltage(voltage * 0.8);
  }

  public boolean objectPresent(){
    return colorSensor.getProximity() > 125;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
 }
}
