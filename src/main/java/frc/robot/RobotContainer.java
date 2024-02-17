// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.autos.FireIntoAmp;
import frc.robot.commands.autos.FireIntoSubwoofer;
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
    configureDirectPowerControllerBindings();
    configureDriverShuffleboard();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);

    Command intakeUntilNoteIsIn = intake.intakeUntilNoteIsIn();
    controller.b().whileTrue(intakeUntilNoteIsIn);

    Command fireIntoSubwoofer = new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight);
    controller.x().whileTrue(fireIntoSubwoofer);

    Command fireIntoAmp = new FireIntoAmp(intake, shooter, shooterLift, drivetrain, limelight);
    controller.y().whileTrue(fireIntoAmp);

    Command holdOntoChains = shooterLift.getGoToPositionCommand(0);
    controller.a().whileTrue(holdOntoChains);

    Command reverseIntake = intake.reverseIntake();
    controller.leftTrigger().whileTrue(reverseIntake);
  }

  private void configureDirectPowerControllerBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);

    Command bringLiftDown = shooterLift.getGoToPowerCommand(-0.1);
    Command bringLiftUp = shooterLift.getGoToPowerCommand(0.1);
    controller.y().whileTrue(bringLiftUp);
    controller.a().whileTrue(bringLiftDown);

    Command intakeIn = intake.getGoToPowerCommand(0.77);
    Command intakeOut = intake.getGoToPowerCommand(-0.77);
    controller.b().whileTrue(intakeIn);
    controller.x().whileTrue(intakeOut);

    Command shoot = shooter.getGoToPowerCommand(0.3);
    Command shootReverse = shooter.getGoToPowerCommand(-0.3);
    controller.povUp().whileTrue(shoot);
    controller.povDown().whileTrue(shootReverse);
  }

  private void configureDriverShuffleboard() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.add(autoChooser);
    setUpAutoChooser(autoChooser);

    driverTab.addBoolean("Has Target To Aim", limelight::hasTarget);
    driverTab.add(CameraServer.startAutomaticCapture());

    driverTab.addString("Currently seen april tag", limelight::getSeenAprilTagAsString);
  }

  private void setUpAutoChooser(SendableChooser<Command> autChooser) {
    autoChooser.addOption("Middle Auto", new MiddleAuto(intake, shooter, shooterLift, drivetrain, limelight));
    autoChooser.addOption("Sideways", new SideAuto(intake, shooter, shooterLift, drivetrain, limelight));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
