// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
/** shooter - two krakens on a ratio of 1:1, 
spin the same way at the same time on the press of a button, pid for speed */
public class Shooter extends SubsystemBase{

    // ADD ENCODER AND WAY TO READ ENCODER VALUES
    private final TalonFX motorOne = new TalonFX(ShooterConstants.kShooterMotorOneId);
    private final TalonFX motorTwo = new TalonFX(ShooterConstants.kShooterMotorTwoId);

    public Shooter() {}

    public void setMotors(double speed){
        motorOne.set(speed);
        motorTwo.set(speed);
    }
}