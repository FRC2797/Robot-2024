package frc.robot.commands;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class AimWithLimelight extends DeferredCommand {
    public AimWithLimelight(SwerveDrivetrain drivetrain, Limelight limelight) {
        super(() -> {
            double rotationToBeAimed = drivetrain.getPose().getRotation().getDegrees() + limelight.getHorizontalOffset();
            return drivetrain.driveToRotation(degreesToRadians(rotationToBeAimed));
        }, Set.of(drivetrain));
    }
}
