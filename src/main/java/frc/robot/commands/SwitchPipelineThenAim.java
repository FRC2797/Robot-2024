package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Pipeline;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwitchPipelineThenAim extends SequentialCommandGroup {
    public SwitchPipelineThenAim(Limelight limelight, SwerveDrivetrain drivetrain, Pipeline pipeline) {
        super(
            limelight.switchPipelineCommand(pipeline),
            new AimWithLimelight(drivetrain, limelight)
        );
    }
}
