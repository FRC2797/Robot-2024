package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class FireNote extends SequentialCommandGroup {
    public FireNote(double height, double shooterRPM, Intake intake, Shooter shooter, ShooterLift shooterLift) {
        super(
            race(
                shooterLift.getGoToPositionCommand(height),
                shooter.getGoToRPMCommand(shooterRPM),
                sequence(
                    waitUntil(() -> shooterLift.atSetpoint()),
                    waitUntil(() -> shooter.atSetpoint()),
                    intake.intakeIntoShooter()
                )
            )
        );
    }

    public FireNote() {
        super(none());
    }
}
