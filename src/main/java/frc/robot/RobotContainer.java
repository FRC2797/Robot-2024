// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.autos.MiddleAuto;
import frc.robot.commands.autos.SideAuto;
import frc.robot.controllers.CommandJoystick;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotContainer {
  Navx navx = new Navx();
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  Limelight limelight = new Limelight();
  ShooterLift shooterLift = new ShooterLift();
  Intake intake = new Intake();
  CommandJoystick joystick = new CommandJoystick(0);
  CommandXboxController controller = new CommandXboxController(0);
  Shooter shooter = new Shooter();
  ShuffleboardTab commandsForTesting = Shuffleboard.getTab("Commands for testing");
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  SwerveJoystick joystickTeleCommand = new SwerveJoystick(
    drivetrain,
    navx,
    () -> controller.getLeftY() * -1,
    () -> controller.getLeftX() * -1,
    () -> controller.getRightX() * -1,
    () -> true
  );


  public RobotContainer() {
    configureBindings();
    configureDriverShuffleboard();
    configureCommandsForTesting();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);
    controller.rightBumper().whileTrue(intake.intakeUntilNoteIsIn());
    controller.b().whileTrue(intake.intake(0.15));
  }

  private void configureDriverShuffleboard() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.add(autoChooser);
    setUpAutoChooser(autoChooser);


    Command realignWheelsForward = run(() -> drivetrain.arcadeDrive(0.1, 0), drivetrain).withTimeout(0.3);
    commandsForTesting.add(realignWheelsForward.withName("Realign wheels Forward"));
    commandsForTesting.add(runOnce(() -> drivetrain.resetOdometry(new Pose2d())).withName("Reset odometry"));

    driverTab.addBoolean("Has Target To Aim", limelight::hasTarget);
    if (false) {
      driverTab.add(CameraServer.startAutomaticCapture());
    }
  }

  private void setUpAutoChooser(SendableChooser<Command> autChooser) {
    autoChooser.addOption("Middle Auto", new MiddleAuto(intake, shooter, shooterLift, drivetrain, limelight));
    autoChooser.addOption("Sideways", new SideAuto(intake, shooter, shooterLift, drivetrain, limelight));
  }

  public void configureCommandsForTesting() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
