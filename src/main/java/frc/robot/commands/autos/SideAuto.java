package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import static edu.wpi.first.math.util.Units.feetToMeters;


public class SideAuto extends SequentialCommandGroup {
    public SideAuto(SwerveDrivetrain drivetrain, Limelight limelight) {
        super(
            new AimWithLimelight(drivetrain, limelight),
            new FireNote(),
            new DriveDistance(feetToMeters(3.5), drivetrain),
            new AimWithLimelight(drivetrain, limelight),
            new FireNote()
        );
    }
}