package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;

public class FireNote extends ParallelDeadlineGroup {
    public FireNote(double height, double leftShooterRPM, double rightShooterRPM, Intake intake, Shooter shooter, ShooterLift shooterLift) {
        super(
            sequence(
                onlyIfAboveAtRest(waitUntil(() -> shooterLift.atGoal()), height, shooterLift),
                waitUntil(() -> shooter.atSetpoint()),
                intake.intakeIntoShooter()
            ),
            onlyIfAboveAtRest(shooterLift.getGoToPositionCommand(height), height, shooterLift),
            shooter.getGoToRPMCommand(leftShooterRPM, rightShooterRPM)
        );
    }

    private static Command onlyIfAboveAtRest(Command cmd, double height, ShooterLift shooterLift) {
        return either(cmd, none(), () -> (height > ShooterLift.atRest.in(Degrees) + 0.1));
    } 
}
