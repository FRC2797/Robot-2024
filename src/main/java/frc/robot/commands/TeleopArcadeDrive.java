package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.applyDeadband;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TeleopArcadeDrive extends Command {
    private CommandJoystick joystick;
    private Drivetrain drivetrain;

    private static double transformStickInput(double stickInput) {
        final double DEADBAND = 0.05;
        stickInput = applyDeadband(stickInput, DEADBAND);
        stickInput *= -1;
        boolean isNegative = stickInput < 0;
        stickInput *= stickInput;
        stickInput = isNegative ? -stickInput : stickInput;
        return stickInput;
    }

    static final double DRIVE_POS_ACCEL_LIM_PER_SEC = 0.5;
    static final double DRIVE_NEG_ACCEL_LIM_PER_SEC = -0.33;

    static final SlewRateLimiter forwardLimiter = new SlewRateLimiter(
            DRIVE_POS_ACCEL_LIM_PER_SEC,
            DRIVE_NEG_ACCEL_LIM_PER_SEC,
            0);

    @Override
    public void execute() {
        double Y = joystick.getY();
        double X = joystick.getX();
        double transformedY = transformStickInput(Y);
        double transformedX = transformStickInput(X);

        drivetrain.arcadeDrive(
                forwardLimiter.calculate(transformedY),
                transformedX);
    }

    public TeleopArcadeDrive(Drivetrain drivetrain, CommandJoystick joystick) {
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        addRequirements(drivetrain);
    }
}
