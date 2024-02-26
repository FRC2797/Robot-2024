// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.autos.FireIntoAmp;
import frc.robot.commands.autos.FireIntoSubwoofer;
import frc.robot.commands.autos.FireNote;
import frc.robot.commands.autos.MiddleAuto;
import frc.robot.commands.autos.SideAuto;
import frc.robot.controllers.CommandJoystick;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLift;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Winch;


public class RobotContainer {
  Navx navx = new Navx();
  Limelight limelight = new Limelight();
  SwerveDrivetrain drivetrain = new SwerveDrivetrain(limelight);
  ShooterLift shooterLift = new ShooterLift();
  Intake intake = new Intake();
  Shooter shooter = new Shooter();
  Winch winch = new Winch();
  
  CommandJoystick joystick = new CommandJoystick(0);
  CommandXboxController controller = new CommandXboxController(0);
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

    Command intakeUntilNoteIsIn = intake.intakeUntilNoteIsIn();
    controller.b().onTrue(intakeUntilNoteIsIn);

    Command fireIntoSubwoofer = new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight);
    controller.x().whileTrue(fireIntoSubwoofer);

    Command fireIntoAmp = new FireIntoAmp(intake, shooter, shooterLift, drivetrain, limelight);
    controller.y().whileTrue(fireIntoAmp);

    Command holdOntoChains = shooterLift.getGoToPositionCommand(0);
    controller.a().whileTrue(holdOntoChains);

    Command reverseIntake = intake.reverseIntake();
    controller.leftTrigger().whileTrue(reverseIntake);
  }

  private void configureClimbBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);
    controller.y().toggleOnTrue(shooterLift.getGoToPositionCommand(1));
    controller.a().toggleOnTrue(shooterLift.getGoToPositionCommand(0.1));
  }

  private void configureDirectPowerControllerBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);

    Command bringLiftDown = shooterLift.getGoToPowerCommand(Volts.of(-2));
    Command bringLiftUp = shooterLift.getGoToPowerCommand(Volts.of(2));
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

    Command winchUp = winch.getGoToPowerCommand(0.1);
    Command winchDown = winch.getGoToPowerCommand(-0.1);
    controller.povRight().whileTrue(winchUp);
    controller.povLeft().whileTrue(winchDown);
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

  private void configureCommandsForTesting() {
    commandsForTesting.add("Aim with limelight", new AimWithLimelight(drivetrain, limelight));
    commandsForTesting.add("Fire into amp", new FireIntoAmp(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Fire into subwoofer", new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Middle auto", new MiddleAuto(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Side Auto", new SideAuto(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Fire Note", new FireNote(0.5, 2000, intake, shooter, shooterLift));

    Command firingWhenDirectlyUpToSubwoofer = new FireNote(0, 2000, intake, shooter, shooterLift).withName("firingWhenDirectlyUpToSubwoofer");
    commandsForTesting.add(firingWhenDirectlyUpToSubwoofer);

    NamedCommands.registerCommand("FireNoteIntoSubwoofer", new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight));
    NamedCommands.registerCommand("IntakeUntilNoteIsIn", intake.intakeUntilNoteIsIn());

    commandsForTesting.add("Path planner middle auto", new PathPlannerAuto("Middle"));
    commandsForTesting.add("Run sysid quasic static on right forward", shooter.sysIdQuasistaticForRight(SysIdRoutine.Direction.kForward));
    commandsForTesting.add("Run sysid quasic static on right backwards", shooter.sysIdQuasistaticForRight(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
