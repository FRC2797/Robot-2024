package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Limelight.Pipeline;

public class SwitchPipelineThenAim extends SequentialCommandGroup {
    public SwitchPipelineThenAim(Limelight limelight, SwerveDrivetrain drivetrain, Pipeline pipeline) {
        super(
            limelight.switchPipelineCommand(pipeline),
            new AimWithLimelight(drivetrain, limelight)
        );
    }
}
