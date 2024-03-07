package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;

public class FireNote extends ParallelDeadlineGroup {
    public FireNote(double height, double shooterRPM, Intake intake, Shooter shooter, ShooterLift shooterLift) {
        super(
            sequence(
                waitUntil(() -> shooterLift.atGoal()),
                waitUntil(() -> shooter.atSetpoint()),
                intake.intakeIntoShooter()
            ),
            shooterLift.getGoToPositionCommand(height),
            shooter.getGoToRPMCommand(shooterRPM)
        );
    }
}
