// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.TeleopArcadeDrive;
import frc.robot.controllers.CommandJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;

public class RobotContainer {
  Drivetrain drivetrain = new Drivetrain();
  Limelight limelight = new Limelight();
  Navx navx = new Navx();
  CommandJoystick joystick = new CommandJoystick(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Command driveForward20Inches = new DriveDistance(20, drivetrain);
    driveForward20Inches.schedule();
    drivetrain.setDefaultCommand(new TeleopArcadeDrive(drivetrain, joystick));
    joystick.five.toggleOnTrue(new AimWithLimelight(drivetrain, limelight));
    joystick.six.toggleOnTrue(new DriveRotation(180, navx, drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  } 
}
