package frc.robot.commands;

import static java.lang.Math.abs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveDistance extends Command {
    private SwerveDrivetrain drivetrain;
    private double distanceToDriveInMeters;

    final double PROP_TERM = 0.004;
    final double TOLERANCE = 0.2;
    final double MIN_TERM;

    private double distanceAlreadyDriven = 0;
    private double distanceDriven = 0;

    public DriveDistance(double distanceToDriveInMeters, SwerveDrivetrain drivetrain) {
        this.distanceToDriveInMeters = distanceToDriveInMeters;
        this.drivetrain = drivetrain;
        this.MIN_TERM = distanceToDriveInMeters > 0 ? 0.05 : -0.05;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        distanceAlreadyDriven = drivetrain.getDistanceDrivenInMeters();
    }

    @Override
    public void execute() {
        distanceDriven =  drivetrain.getDistanceDrivenInMeters() - distanceAlreadyDriven;
        double error = distanceToDriveInMeters - distanceDriven;
        double speed = (error * PROP_TERM) + MIN_TERM;

        drivetrain.arcadeDrive(speed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
        distanceAlreadyDriven = drivetrain.getDistanceDrivenInMeters();
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(distanceToDriveInMeters, distanceDriven, TOLERANCE);
    }
}
