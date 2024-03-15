// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Arm armSubsystem = new Arm();
  private final Intake intakeSubsystem = new Intake();
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
    // button to raise arm
    driverJoystick.triangle().whileTrue(new SetArmAngle(armSubsystem, armSubsystem.getAngle() + 1));
    // button to lower arm
    driverJoystick.cross().whileTrue(new SetArmAngle(armSubsystem, armSubsystem.getAngle() - 1));
    // example button to set to a given angle
    driverJoystick.circle().whileTrue(new SetArmAngle(armSubsystem, ArmConstants.shootingAngle));
    // button to turn on the intake
    driverJoystick.circle().whileTrue(new SetIntake(intakeSubsystem, false));
    driverJoystick.cross().whileTrue(new SetIntake(intakeSubsystem, true));
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
