package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.AimWithLimelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;
import frc.robot.subsystems.SwerveDrivetrain;

public class FireIntoSubwoofer extends DeferredCommand {
    final private static Measure<Distance> kDistanceToFireAt = Inches.of(90);
    final private static double kHeightToFireAt = 0.8;
    final private static double kShooterRPMToFireAt = 2000;

    public FireIntoSubwoofer(
        Intake intake,
        Shooter shooter,
        ShooterLift shooterLift,
        SwerveDrivetrain drivetrain,
        Limelight limelight
    ) {
        super(() -> {
            Measure<Distance> currentDistance = limelight.getDistance();
            Measure<Distance> distanceToDrive = currentDistance.minus(kDistanceToFireAt);
            
            // because the limelight is on the back
            distanceToDrive = distanceToDrive.times(-1);
            return 
                sequence(
                    new AimWithLimelight(drivetrain, limelight),
                    drivetrain.driveDistanceWithJustPID(distanceToDrive.in(Meters)),
                    new FireNote(kHeightToFireAt, kShooterRPMToFireAt, intake, shooter, shooterLift)
                );
        }, Set.of(intake, shooter, shooterLift, drivetrain));
    }
}
