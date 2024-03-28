package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StopShooting extends Command {
    private final Shooter shooter;
    private final Intake  intake;

    public StopShooting(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
    }


    @Override
    public void initialize(){
        shooter.setMotors(0);
        intake.setMotor(0);
    }

    @Override
    public void execute(){
}

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}