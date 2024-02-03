package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLift;

public class ShooterUp extends Command {

    private ShooterLift shooterLift;
    private double speed;

    public ShooterUp(ShooterLift shooterLift, double speed) {
        this.shooterLift = shooterLift;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        shooterLift.enableMotors(true);
    }

    @Override
    public void execute() {
        shooterLift.shooterUp(speed);    
    }

    @Override 
    public void end(boolean interrupted){
        shooterLift.shooterStop();
        shooterLift.disableMotors(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
