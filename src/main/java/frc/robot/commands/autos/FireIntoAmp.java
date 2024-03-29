package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithLimelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;
import frc.robot.subsystems.SwerveDrivetrain;

public class FireIntoAmp extends SequentialCommandGroup {
    final private static double kHeightToFireAt = 0.95;
    final private static double kShooterRPMToFireAt = 500;

    public FireIntoAmp(Intake intake, Shooter shooter, ShooterLift shooterLift, SwerveDrivetrain drivetrain, Limelight limelight) {
        super(
            new AimWithLimelight(drivetrain, limelight),
            new FireNote(kHeightToFireAt, kShooterRPMToFireAt, kShooterRPMToFireAt, intake, shooter, shooterLift)
        );
    }
}
