package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class AimWithLimelight extends DeferredCommand {
    public AimWithLimelight(SwerveDrivetrain drivetrain, Limelight limelight) {
        super(() -> {
            double rotationToBeAimed = drivetrain.getPose().getRotation().getDegrees() + limelight.getHorizontalOffset();
            return drivetrain.driveToPoseRelativeToCurrent(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(rotationToBeAimed)), true);
        }, Set.of(drivetrain));
    }
}
