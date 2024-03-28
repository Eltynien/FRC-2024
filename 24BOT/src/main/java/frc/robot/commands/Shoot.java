package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private final PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    private final Shooter shooter;
    private final Intake  intake;

    public Shoot(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        shooter.setMotors(pid.calculate(shooter.getEncoderVelocity(), 250));
        SmartDashboard.putNumber("shooter", shooter.getEncoderVelocity());
        if (Math.abs(shooter.getEncoderVelocity()) > 50){
            intake.setMotor(7);
        }
    }

    @Override
    public void end(boolean interrupted){
        shooter.setMotors(0);
        intake.setMotor(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}