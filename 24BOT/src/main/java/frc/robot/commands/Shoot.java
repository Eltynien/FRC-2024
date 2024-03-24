package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private final PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    private Shooter shooter;

    public Shoot(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        shooter.setMotors(pid.calculate(5.0, shooter.getEncoderPosition()));

    }

    @Override
    public void end(boolean interrupted){
        shooter.setMotors(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}