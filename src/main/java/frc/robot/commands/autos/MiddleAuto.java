package frc.robot.commands.autos;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.wpilibj2.command.Commands.race;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;
import frc.robot.subsystems.SwerveDrivetrain;

public class MiddleAuto extends SequentialCommandGroup {
    public MiddleAuto(
        Intake intake,
        Shooter shooter,
        ShooterLift shooterLift,
        SwerveDrivetrain drivetrain,
        Limelight limelight
    ) {
        super(
            new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight),
            race(
                intake.intake(),
                shooterLift.getGoToPositionCommand(0),
                drivetrain.driveDistanceWithJustPID(feetToMeters(6))
            ),
            new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight)
        );
    }
}
