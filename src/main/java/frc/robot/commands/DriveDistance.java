package frc.robot.commands;

import static java.lang.Math.abs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends Command {
    private Drivetrain drivetrain;
    private double distanceToDrive;

    final double PROP_TERM = 0.004;
    final double MIN_TERM;

    public DriveDistance(double distanceToDrive, Drivetrain drivetrain) {
        this.distanceToDrive = distanceToDrive;
        this.drivetrain = drivetrain;
        this.MIN_TERM = distanceToDrive > 0 ? 0.05 : -0.05;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        double distanceDriven = drivetrain.getDistanceDrivenInInches();
        double error = distanceToDrive - distanceDriven;
        double speed = (error * PROP_TERM) + MIN_TERM;

        drivetrain.arcadeDrive(speed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
        drivetrain.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return abs(drivetrain.getDistanceDrivenInInches()) > abs(distanceToDrive);
    }
}
