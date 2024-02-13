package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;
import frc.robot.subsystems.SwerveDrivetrain;

public class GoToDistanceAndFireNote extends DeferredCommand {
    final private static Measure<Distance> kDistanceToFireAt = Inches.of(45);
    final private static double kHeightToFireAt = 0.8;
    final private static double kShooterRPMToFireAt = 2000;

    public GoToDistanceAndFireNote(
        Intake intake,
        Shooter shooter,
        ShooterLift shooterLift,
        SwerveDrivetrain drivetrain,
        Limelight limelight
    ) {
        super(() -> {
            Measure<Distance> currentDistance = limelight.getDistance();
            Translation2d translation = new Translation2d(currentDistance.minus(kDistanceToFireAt), Inches.of(0));
            return 
                drivetrain.driveToPoseRelativeToCurrent(new Pose2d(translation, new Rotation2d()), true)
                .andThen(
                    new FireNote(kHeightToFireAt, kShooterRPMToFireAt, intake, shooter, shooterLift)
                );
        }, Set.of(intake, shooter, shooterLift));
    }
}
