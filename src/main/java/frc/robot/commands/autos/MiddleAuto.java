package frc.robot.commands.autos;

import static edu.wpi.first.math.util.Units.feetToMeters;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class MiddleAuto extends SequentialCommandGroup {
    public MiddleAuto(SwerveDrivetrain drivetrain, Limelight limelight) {
        super(
            new AimWithLimelight(drivetrain, limelight),
            new FireNote(),
            new DriveDistance(feetToMeters(6), drivetrain),
            new AimWithLimelight(drivetrain, limelight),
            new FireNote()
        );
    }
}
