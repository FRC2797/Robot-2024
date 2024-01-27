// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.autos.MiddleAuto;
import frc.robot.commands.autos.SideAuto;
import frc.robot.controllers.CommandJoystick;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotContainer {
  Navx navx = new Navx();
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  Limelight limelight = new Limelight();
  CommandJoystick joystick = new CommandJoystick(0);
  CommandXboxController controller = new CommandXboxController(0);
  ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  SwerveJoystick joystickTeleCommand = new SwerveJoystick(
    drivetrain,
    navx,
    () -> controller.getLeftY() * -1,
    controller::getLeftX,
    controller::getRightX,
    () -> true
  );


  public RobotContainer() {
    configureBindings();
    configureDriverShuffleboard();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);
  }

  private void configureDriverShuffleboard() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.add(autoChooser);

    autoChooser.addOption("Middle Auto", new MiddleAuto(drivetrain, limelight));
    autoChooser.addOption("Sideways Auto", new SideAuto(drivetrain, limelight));
    autoChooser.addOption("Move Forward a meter", new DriveDistance(1, drivetrain));

    autoChooser.addOption("Rotate 360deg", new DriveRotation(360, navx, drivetrain));

    Command realignWheelsForward = run(() -> drivetrain.arcadeDrive(0.1, 0), drivetrain).withTimeout(0.3);
    commandsTab.add(realignWheelsForward.withName("Realign wheels Forward"));

    Command moveForwardAndComeBack = sequence(
      new DriveDistance(2, drivetrain),
      new DriveRotation(180, navx, drivetrain),
      new DriveDistance(2, drivetrain)
    ).withName("Move Forward and come back");

    autoChooser.addOption("Move forward and come back", moveForwardAndComeBack);

    Command moveBackward = new DriveDistance(-1, drivetrain).withName("Move Backward");

    autoChooser.addOption("Move backward", moveBackward);

    driverTab.addBoolean("Has Target To Aim", limelight::hasTarget);
    if (false) {
      driverTab.add(CameraServer.startAutomaticCapture());
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
