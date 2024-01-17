// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.TeleopArcadeDrive;
import frc.robot.controllers.CommandJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class RobotContainer {
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  Limelight limelight = new Limelight();
  CommandJoystick joystick = new CommandJoystick(0);
  CommandXboxController controller = new CommandXboxController(0);

  SwerveJoystick joystickTeleCommand = new SwerveJoystick(
    drivetrain,
    controller::getLeftY,
    controller::getLeftX,
    controller::getRightX,
    () -> true
  );


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(joystickTeleCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
