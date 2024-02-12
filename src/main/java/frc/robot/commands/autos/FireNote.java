package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class FireNote extends ParallelRaceGroup {
    public FireNote(double height, double shooterRPM, Intake intake, Shooter shooter, ShooterLift shooterLift) {
        super(
            shooterLift.getGoToPositionCommand(height),
            shooter.getGoToRPMCommand(shooterRPM),
            sequence(
                waitUntil(() -> shooterLift.atSetpoint()),
                waitUntil(() -> shooter.atSetpoint()),
                intake.intakeIntoShooter()
            )
        );
    }

    public FireNote() {
        super(none());
    }
}
