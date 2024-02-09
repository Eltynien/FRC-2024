// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final CommandPS4Controller driverJoystick = new CommandPS4Controller(OIConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(swerveSubsystem);

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveSubsystem,
        () -> driverJoystick.getRightY(),
        () -> -driverJoystick.getRightX(),
        () -> driverJoystick.getLeftX()
      ));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
  }


  private void configureBindings() {
    //driverJoystick.circle().whileTrue(new RaiseArmToAngle(45, arm));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
