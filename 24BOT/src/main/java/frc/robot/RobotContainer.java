// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.StopShooting;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final LimeLight limelightSubsystem = new LimeLight();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Arm armSubsystem = new Arm();
  private final Intake intakeSubsystem = new Intake();
  private final Shooter shooterSubsystem = new Shooter();
  private final LimeLight limeLight = new LimeLight();
  private final CommandGenericHID driverJoystick1 = new CommandGenericHID(OIConstants.kDriverControllerPort1);
  private final CommandPS4Controller driverJoystick2 = new CommandPS4Controller(OIConstants.kDriverControllerPort2);
  private final SendableChooser<Command> autoChooser;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    limeLight.turnOffLED();
    swerveSubsystem.setDefaultCommand(
      //driverJoystick1.getRawAxis()
      new SwerveJoystickCommand(
        swerveSubsystem,
        limeLight,
        () -> driverJoystick1.getRawAxis(1),
        () -> -driverJoystick1.getRawAxis(0),
        () -> driverJoystick1.getRawAxis(3),
        () -> driverJoystick1.getRawAxis(6)
      )
    );

    // register named commands with the autonomous builder
    NamedCommands.registerCommand( // USE IN PARALLLEL WITH PATH
      "Intake",  
      new SequentialCommandGroup(
        new SetArmAngle(armSubsystem, ArmConstants.kIntakeAngle),
        new SetIntake(intakeSubsystem, false)
      ));
    NamedCommands.registerCommand("Shoot", 
      new SequentialCommandGroup(
        new SetArmAngle(armSubsystem, ArmConstants.shootingAngle),
        new ParallelDeadlineGroup(
          new WaitCommand(0.5),
          new Shoot(shooterSubsystem, intakeSubsystem)
        ),
        new StopShooting(shooterSubsystem, intakeSubsystem)
      )
    );
   
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }


  private void configureBindings() {
    // button to raise to shooting angle
    driverJoystick2.square().onTrue(new SetArmAngle(armSubsystem, ArmConstants.shootingAngle));
    // button to raise to intake angle
    driverJoystick2.triangle().onTrue(new SetArmAngle(armSubsystem, ArmConstants.kIntakeAngle));
    // button to raise to be flat
    driverJoystick1.axisGreaterThan(7, 0).onTrue(new SetArmAngle(armSubsystem, ArmConstants.minAngle));
    // button to raise to be vertical
    driverJoystick1.axisLessThan(7, 0).onTrue(new SetArmAngle(armSubsystem, ArmConstants.maxAngle));
    // button to turn on the intake
    driverJoystick1.button(2).whileTrue(
      new SequentialCommandGroup(
          new SetArmAngle(armSubsystem, ArmConstants.minAngle),
          new SetIntake(intakeSubsystem, false)
      )
    );
    // button to reverse the intake
    driverJoystick2.circle().whileTrue(new SetIntake(intakeSubsystem, true));
    //button to shoot
    driverJoystick1.button(1).onTrue(
      new SetArmAngle(armSubsystem, limeLight)
    );

    driverJoystick1.button(1).whileTrue(
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new RepeatCommand(new Shoot(shooterSubsystem, intakeSubsystem))
      )
    );
    // button to turn towards target 
    //driverJoystick1.axisGreaterThan(4,0).onTrue(new TurnToTarget(limelightSubsystem, swerveSubsystem));

    // button to reset odometry
    driverJoystick2.cross().onTrue(new InstantCommand(() -> swerveSubsystem.resetGyroscope(), swerveSubsystem));
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
