package frc.robot.commands;

import static java.lang.Math.abs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class AimWithLimelight extends Command {
    final double TOLERANCE = 1;
    final double SPEED = 0.08;
    final private SwerveDrivetrain drivetrain;
    final private Limelight limelight;

    public AimWithLimelight(SwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double horizontalOffset = limelight.getHorizontalOffset();

        if (horizontalOffset > TOLERANCE) {
          drivetrain.arcadeDrive(0, -SPEED);
        } else if (horizontalOffset < TOLERANCE) {
          drivetrain.arcadeDrive(0, SPEED);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return abs(limelight.getHorizontalOffset()) < TOLERANCE;
    }
}
