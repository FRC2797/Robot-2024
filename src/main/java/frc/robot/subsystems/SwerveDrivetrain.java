// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.io.File;
import java.util.function.DoubleConsumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


// copy and pasted from https://github.com/BroncBotz3481/YAGSL-Example while adding our own methods
public class SwerveDrivetrain extends SubsystemBase
{

  ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
  private final SwerveDrive swerveDrive;
  public        double      maximumSpeed = Constants.ModuleConstants.kPhysicalMaxSpeedMetersPerSecond;
  private Limelight limelight;

  public SwerveDrivetrain(File directory, Limelight limelight)
  {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    
    setupPathPlanner();

    tab.addString("The robot pose is", () -> swerveDrive.getPose().toString());
    tab.addDouble("The current rotation in degrees is ", () -> swerveDrive.getPose().getRotation().getDegrees());
    tab.addString("One of the modules", () -> swerveDrive.getModulePositions()[0].toString());
    tab.add("Reset odometry", runOnce(() -> resetOdometry(new Pose2d())));
    tab.add("Drive to rotation 0", driveToRotation(0));
    tab.add("Drive 1 meter with just pid", driveDistanceWithJustPID(1).withName("Drive 1 meter with just pid"));
    tab.add("Drive -1 meter with just pid", driveDistanceWithJustPID(-1).withName("Drive -1 meter with just pid"));

    swerveDrive.setHeadingCorrection(true);

    this.limelight = limelight;
  }

  public SwerveDrivetrain(Limelight limelight) {
    this(
      new File(Filesystem.getDeployDirectory(), "swerve"),
      limelight
    );
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         getTranslationConstants(),
                                         // Translation PID constants
                                         new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                                          swerveDrive.swerveController.config.headingPIDF.i,
                                                          swerveDrive.swerveController.config.headingPIDF.d),
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @param setOdomToStart Set the odometry position to the start of the path.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    if (setOdomToStart)
    {
      resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
    }

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        getPathConstraints(),
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                                     );
  }

  public PathConstraints getPathConstraints() {
    return new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));
  }

  public Command driveToPoseRelativeToCurrent(Pose2d pose, boolean canFinishRotation) {
    return defer(() -> {
      return driveToPose(pose.relativeTo(getPose()), canFinishRotation);
    });
  }

  public Command driveToPose(Pose2d pose, boolean canFinishRotation) {
    if (canFinishRotation) {
      return driveToPose(pose).andThen(driveToRotation(pose.getRotation().getRadians()));
    } else {
      return driveToPose(pose);
    }
  }

  public Command driveToRotation(double radians) {
    PIDController controller = swerveDrive.swerveController.config.headingPIDF.createPIDController();
    controller.enableContinuousInput(-Math.PI, Math.PI);

    controller.setTolerance(degreesToRadians(1));

    DoubleConsumer rotate = (output) -> {
      swerveDrive.drive(new Translation2d(), output, false, false, new Translation2d());
      SmartDashboard.putNumber("Current drive to rotation output", output);
    };

    return new PIDCommand(
      controller,
      () -> getPose().getRotation().getRadians(),
      radians,
      rotate,
      this
    ).until(controller::atSetpoint);
  }

  public Command driveToRotationRelative(double radians) {
    return driveToRotation(getPose().getRotation().getRadians() + radians);
  }

  public Command driveDistanceUsingPoses(double meters) {
    return driveToPoseRelativeToCurrent(
      new Pose2d(new Translation2d(meters, 0), new Rotation2d()),
      false
    );
  }

  // Should be using the driveToPose but it wasn't working
  public Command driveDistanceWithJustPID(double meters) {
    PIDConstants constants = getTranslationConstants();
    PIDController controller = new PIDController(constants.kP, constants.kI, constants.kD);

    controller.setTolerance(0.1);

    DoubleConsumer drive = (output) -> {
      swerveDrive.drive(new Translation2d(output, 0), 0, false, false, new Translation2d());
    };

    return defer(
      () -> {
        Translation2d originalTranslation = getPose().getTranslation();
        return new PIDCommand(
          controller,
          () -> originalTranslation.getDistance(getPose().getTranslation()) * Math.signum(meters),
          meters,
          drive,
          this
        ).until(controller::atSetpoint);
      }
    );
  }


  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  public void addLimelightReadingToOdometry() {
    if (limelight.hasTarget()) {
      Pose2d limelightPose = limelight.getBosePose2d_wpiBlue();

      boolean poseIsTooOff = getPose().getTranslation().getDistance(limelightPose.getTranslation()) > 1;
      if (poseIsTooOff) {
        return;
      } else {
        swerveDrive.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - limelight.getLatency());
      }
    }
  }

  public void arcadeDrive(double forwardInMetersPerSecond, double rotationInRadians) {
    var chassisSpeeds = new ChassisSpeeds(forwardInMetersPerSecond, 0, rotationInRadians);
    this.drive(chassisSpeeds);
  }

  public double getDistanceDrivenInMeters() {
    return swerveDrive.getPose().getX();
  }

  private PIDConstants getTranslationConstants() {
    return new PIDConstants(5.0, 0.0, 0.0);
  }
}