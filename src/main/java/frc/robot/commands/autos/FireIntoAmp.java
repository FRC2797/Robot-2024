package frc.robot.commands.autos;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;

public class FireIntoAmp extends FireNote {
    final private static double kHeightToFireAt = 0.95;
    final private static double kShooterRPMToFireAt = 500;

    public FireIntoAmp(Intake intake, Shooter shooter, ShooterLift shooterLift) {
        super(
            kHeightToFireAt,
            kShooterRPMToFireAt,
            intake,
            shooter,
            shooterLift
        );
    }
}
