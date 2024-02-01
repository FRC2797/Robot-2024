package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLift;

public class ShooterDown extends Command {

    private ShooterLift shooterLift;
    private double speed;

    public ShooterDown(ShooterLift shooterLift, double speed) {
        this.shooterLift = shooterLift;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shooterLift.shooterDown(speed);    
    }

    @Override 
    public void end(boolean interrupted){
        shooterLift.shooterStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
