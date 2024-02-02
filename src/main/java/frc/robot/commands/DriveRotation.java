package frc.robot.commands;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveRotation extends Command {
    private final double PROP_TERM = 0.012;
    private final double MIN_TERM = 0.3;
    private final double TOLERANCE = 1;

    private final SwerveDrivetrain drivetrain;

    final double degreesToRotate;
    private double initialYaw;
    private double distanceRotated = 0;

    private Navx navx;

    public DriveRotation(double degreesToRotate, Navx navx, SwerveDrivetrain drivetrain) {
        this.navx = navx;
        this.degreesToRotate = degreesToRotate;

        this.drivetrain = drivetrain;
        addRequirements(navx, drivetrain);
    }

    @Override
    public void initialize() {
        initialYaw = navx.getAngle();
    }

    @Override
    public void execute() {
        distanceRotated = navx.getAngle() - initialYaw;
        double error = degreesToRotate - distanceRotated;
        double speed = (error * PROP_TERM) + MIN_TERM * signum(error);

        drivetrain.arcadeDrive(0, speed);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Degrees to rotate: " + degreesToRotate);
        System.out.println("distanceRotated: " + distanceRotated);
        return MathUtil.isNear(degreesToRotate, distanceRotated, TOLERANCE);
    }

}
