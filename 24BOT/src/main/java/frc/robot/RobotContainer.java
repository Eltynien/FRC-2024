// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LimeLight limelightSubsystem = new LimeLight();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Arm armSubsystem = new Arm();
  private final Intake intakeSubsystem = new Intake();
  private final Shooter shooterSubsystem = new Shooter();
  private final CommandPS4Controller driverJoystick = new CommandPS4Controller(OIConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(swerveSubsystem);

    swerveSubsystem.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveSubsystem,
        limelightSubsystem,
        () -> -driverJoystick.getLeftY(),
        () -> driverJoystick.getLeftX(),
        () -> -driverJoystick.getRightX(),
        () -> driverJoystick.getL2Axis()
      ));

    // register named commands with the autonomous builder
  
    NamedCommands.registerCommand("SetArmAngle", new SetArmAngle(armSubsystem, limelightSubsystem));
    NamedCommands.registerCommand("SetIntakeIn", new SetIntake(intakeSubsystem, false));
    NamedCommands.registerCommand("SetIntakeOut", new SetIntake(intakeSubsystem, true));
    NamedCommands.registerCommand("Shoot", new Shoot(shooterSubsystem));
    
   
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }


  private void configureBindings() {
    // button to raise arm
    driverJoystick.L1().whileTrue(new SetArm(armSubsystem, false));

    // button to lower arm
    driverJoystick.L2().whileTrue(new SetArm(armSubsystem, true));

    // example button to set to a given angle/towards any detected target
    //driverJoystick.square().whileTrue(new SetArmAngle(armSubsystem, limelightSubsystem));
    driverJoystick.square().onTrue(new SetArmAngle(armSubsystem, 45));

    // button to turn on the intake
    driverJoystick.R1().whileTrue(new SetIntake(intakeSubsystem, false));

    // button to reverse the intake
    driverJoystick.R2().whileTrue(new SetIntake(intakeSubsystem, true));
    
    //button to shoot
    driverJoystick.circle().whileTrue(new Shoot(shooterSubsystem));
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
