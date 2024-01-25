package frc.robot.subsystems;

import static frc.robot.Constants.showNonessentialShuffleboardInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

// refactor to use navx subsystem. Do by adding a getContinousHeading method
public class SwerveDrivetrain extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed
            );

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed
            );

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed
            );

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed
            );

    private final Navx navx;
    private final SwerveDriveOdometry odometer; 
    private final ShuffleboardTab tab = Shuffleboard.getTab("SwerveDrivetrain");

    public SwerveDrivetrain(Navx navx) {
        this.navx = navx;

        this.odometer = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            navx.getRotation2d(),
            getPositions()
        );

        if (showNonessentialShuffleboardInfo) {
            tab.addDouble("Distance Driven in meters ", this::getDistanceDrivenInMeters);
            tab.addString("Robot Location", () -> getPose().getTranslation().toString());
        }
    }

    @Override
    public void periodic() {
        odometer.update(
            navx.getRotation2d(),
            getPositions()
        );
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(navx.getRotation2d(), getPositions(), pose);
    }

    private SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
        return positions; 
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void arcadeDrive(double forwardInMetersPerSecond, double rotationInRadians) {
        var chassisSpeeds = new ChassisSpeeds(forwardInMetersPerSecond, 0, rotationInRadians);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public double getDistanceDrivenInMeters() {
        return (
            frontLeft.getDrivePosition()
            + frontRight.getDrivePosition()
            + backLeft.getDrivePosition()
            + backRight.getDrivePosition()
        ) / 4;
    }
}