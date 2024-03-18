// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.startEnd;

import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private final SendableChooser<Runnable> controlSchemeChooser = new SendableChooser<>();


  SwerveJoystick joystickTeleCommand = new SwerveJoystick(
    drivetrain,
    navx,
    () -> controller.getLeftY() * -1,
    () -> controller.getLeftX() * -1,
    () -> controller.getRightX() * -1,
    () -> true
  );


  public RobotContainer() {
    configureDriverShuffleboard();
    configureCommandsForTesting();
  }

  private void configureBenControllerScheme() {
    drivetrain.setDefaultCommand(joystickTeleCommand);
    
    controller.rightTrigger().whileTrue(shooter.getGoToRPMCommand(2700, 2200));

    controller.leftTrigger().onTrue(either(intake.intakeUntilNoteIsIn(), none(), () -> shooterLift.isFullyDown()));
    controller.leftTrigger().whileTrue(either(intake.intakeIntoShooter(), none(), () -> !shooterLift.isFullyDown()));

    Command armUp = shooterLift.getGoToPowerCommand(Volts.of(12 * 0.4));
    Command armDown = shooterLift.getGoToPowerCommand(Volts.of(12 * -0.2));
    controller.y().whileTrue(armUp);
    controller.a().whileTrue(armDown);

    controller.rightBumper().whileTrue(parallel(
      shooter.getGoToRPMCommand(4000, 4000),
      shooterLift.getGoToPositionCommand(10)
    ));

    Supplier<Command> unbrakeThenBrakeShooterLiftWithoutShooterliftRequired = () -> startEnd(shooterLift::unbrakeMotors, shooterLift::brakeMotors);
    Command winchAndShooterDown = parallel(
      unbrakeThenBrakeShooterLiftWithoutShooterliftRequired.get(),
      winch.getGoToPowerCommand(0.4),
      shooterLift.getGoToPowerCommand(Volts.of(12 * -0.2))
    );
    controller.povDown().whileTrue(winchAndShooterDown);
  }
  
  private void configureMinimumViableControllerScheme() {
    drivetrain.setDefaultCommand(joystickTeleCommand);
    
    Command intakeIn = intake.getGoToPowerCommand(0.4);
    Command intakeOut = intake.getGoToPowerCommand(-0.4);
    controller.x().whileTrue(intakeIn);
    controller.b().whileTrue(intakeOut);

    Command armUp = shooterLift.getGoToPowerCommand(Volts.of(12 * 0.2));
    Command armDown = shooterLift.getGoToPowerCommand(Volts.of(12 * -0.2));
    controller.y().whileTrue(armUp);
    controller.a().whileTrue(armDown);

    double maxShooterRPM = 3500;
    Command analogShooter = runOnce(() -> shooter.enable()).andThen(run(() -> shooter.setSetpoint(maxShooterRPM * controller.getRightTriggerAxis()), shooter)).finallyDo(() -> shooter.disable());
    controller.rightTrigger(0.01).whileTrue(analogShooter);

    Command shootWhenDirectlyUpAgainstSubwoofer = shooter.getGoToRPMCommand(2400);
    controller.povUp().toggleOnTrue(shootWhenDirectlyUpAgainstSubwoofer);
    controller.povLeft().onTrue(runOnce(() -> shooterLift.unbrakeMotors(), shooterLift));
    controller.povRight().onTrue(runOnce(() -> shooterLift.brakeMotors(), shooterLift));

    Supplier<Command> unbrakeThenBrakeShooterLift = () -> startEnd(shooterLift::unbrakeMotors, shooterLift::brakeMotors, shooterLift);
    
    // spool in
    controller.leftBumper().whileTrue(parallel(
      unbrakeThenBrakeShooterLift.get(),
      winch.getGoToPowerCommand(0.2)
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    Command shoot = shooter.getGoToRPMCommand(1500);
    Command ampShoot = shooterLift.getGoToPositionCommand(115).andThen(shoot);
    controller.rightBumper().onTrue(ampShoot);

    // don't be stupid and forget the shooterLift isn't required
    Supplier<Command> unbrakeThenBrakeShooterLiftWithoutShooterliftRequired = () -> startEnd(shooterLift::unbrakeMotors, shooterLift::brakeMotors);
    Command winchAndShooterDown = parallel(
      unbrakeThenBrakeShooterLiftWithoutShooterliftRequired.get(),
      winch.getGoToPowerCommand(0.4),
      shooterLift.getGoToPowerCommand(Volts.of(12 * -0.2))
    );

    controller.povDown().whileTrue(winchAndShooterDown);
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
    setUpAutoChooser(autoChooser);
    driverTab.add("Auto Chooser", autoChooser);

    setUpControlSchemeChooser(controlSchemeChooser);
    driverTab.add("Control Scheme Chooser", controlSchemeChooser);

    driverTab.add(CameraServer.startAutomaticCapture());
  }

  private void setUpAutoChooser(SendableChooser<Command> autoChooser) {
    Supplier<Command> liftGoToRest = () -> shooterLift.getSetInitialMeasurement().andThen(shooterLift.getGoToRestCommand());
    Supplier<Command> releaseLock = () -> shooterLift.getGoToPowerCommand(Volts.of(1.15)).withTimeout(0.5).andThen(shooterLift.getGoToPowerCommand(Volts.of(-1)).withTimeout(0.5));
    Supplier<Command> sidewaysAuto = () -> sequence(
        releaseLock.get(),
        deadline(liftGoToRest.get(), intake.intake(0.1).until(intake::noteIsIn)),
        new FireNote(10, 4000, 4000, intake, shooter, shooterLift)
    );

    autoChooser.addOption("Middle Auto", 
      sequence(
        defer(() -> drivetrain.resetGyroAtBeginningOfMatch(DriverStation.getAlliance().get(), false, false), Set.of(drivetrain)),
        releaseLock.get(),
        deadline(liftGoToRest.get(), intake.intake(0.1).until(intake::noteIsIn)),
        new FireNote(2, 2700, 2200, intake, shooter, shooterLift),
        deadline(
          intake.intakeUntilNoteIsIn(),
          drivetrain.driveDistanceWithJustPID(inchesToMeters(48))
        ),
        drivetrain.driveDistanceWithJustPID(inchesToMeters(-41)),
        run(() -> drivetrain.drive(new ChassisSpeeds(-0.5, 0.0, 0.0)), drivetrain).withTimeout(0.5),
        new FireNote(2, 2700, 2200 , intake, shooter, shooterLift)
      )  
    );

    autoChooser.addOption("Sideways long side", sidewaysAuto.get().beforeStarting(
      defer(() -> drivetrain.resetGyroAtBeginningOfMatch(DriverStation.getAlliance().get(), true, true), Set.of(drivetrain))
    ));

    autoChooser.addOption("Sideways short side", sidewaysAuto.get().beforeStarting(
      defer(() -> drivetrain.resetGyroAtBeginningOfMatch(DriverStation.getAlliance().get(), true, false), Set.of(drivetrain))
    ));

    autoChooser.setDefaultOption("Nothing", none());
  }

  private void setUpControlSchemeChooser(SendableChooser<Runnable> controlSchemeChooser) { 
    controlSchemeChooser.setDefaultOption("Minimum viable controller scheme", () -> {
      CommandScheduler.getInstance().getActiveButtonLoop().clear();
      configureMinimumViableControllerScheme();
    });

    controlSchemeChooser.addOption("Direct power controller scheme", () -> {
      CommandScheduler.getInstance().getActiveButtonLoop().clear();
      configureDirectPowerControllerBindings();
    });

    controlSchemeChooser.addOption("Ben controller scheme", () -> {
      CommandScheduler.getInstance().getActiveButtonLoop().clear();
      configureBenControllerScheme();
    });
  }

  private void configureCommandsForTesting() {
    commandsForTesting.add("Aim with limelight", new AimWithLimelight(drivetrain, limelight));
    commandsForTesting.add("Fire into amp", new FireIntoAmp(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Fire into subwoofer", new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Middle auto", new MiddleAuto(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Side Auto", new SideAuto(intake, shooter, shooterLift, drivetrain, limelight));
    commandsForTesting.add("Fire Note", new FireNote(30, 2000, 2000, intake, shooter, shooterLift));

    Supplier<Command> fireWhenDirectlyUpToSubwoofer = () -> new FireNote(0, 2700, 2700, intake, shooter, shooterLift);

    commandsForTesting.add("fire when directly up to subwoofer", fireWhenDirectlyUpToSubwoofer.get());

    commandsForTesting.add("basic auto",
      fireWhenDirectlyUpToSubwoofer.get().andThen(
        drivetrain.driveDistanceWithJustPID(Inches.of(50).in(Meters)).alongWith(intake.intakeUntilNoteIsIn())
      ).andThen(
        new FireNote(20, 3500, 3500, intake, shooter, shooterLift)
      ));

    NamedCommands.registerCommand("FireNoteIntoSubwoofer", new FireIntoSubwoofer(intake, shooter, shooterLift, drivetrain, limelight));
    NamedCommands.registerCommand("IntakeUntilNoteIsIn", intake.intakeUntilNoteIsIn());

    commandsForTesting.add("Path planner middle auto", new PathPlannerAuto("Middle"));
    commandsForTesting.add("Run sysid quasic static on right forward", shooter.sysIdQuasistaticForRight(SysIdRoutine.Direction.kForward));
    commandsForTesting.add("Run sysid quasic static on right backwards", shooter.sysIdQuasistaticForRight(SysIdRoutine.Direction.kReverse));

    commandsForTesting.add("Fire side note ", new FireNote(20, 4500, 4500, intake, shooter, shooterLift));
    commandsForTesting.add(CommandScheduler.getInstance());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Runnable getControlScheme() {
    return controlSchemeChooser.getSelected();
  }
}
