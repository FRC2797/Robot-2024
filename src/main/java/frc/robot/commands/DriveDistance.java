package frc.robot.commands;

import static java.lang.Math.abs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends Command {
    private double inches;
    private Drivetrain drivetrain;

    final double PROP_TERM = 0.004;
    final double MIN_TERM = inches > 0 ? 0.05 : -0.05;
    final double distanceToDrive = inches;

    public DriveDistance(double inches, Drivetrain drivetrain) {
        this.inches = inches;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        withName("Drive " + inches + " inches");
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
    public boolean isFinished() {
        return abs(drivetrain.getDistanceDrivenInInches()) > abs(inches);
    }

}
