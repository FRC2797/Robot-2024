// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Robot extends TimedRobot {
  SwerveDrive swerveDrive;

  @Override
  public void robotInit() {
    Preferences.initDouble("vx", 0);
    Preferences.initDouble("vy", 0);
    Preferences.initDouble("omega", 0);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(5);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
  }

  @Override
  public void testPeriodic() {
    swerveDrive.drive(new ChassisSpeeds(
        Preferences.getDouble("vx", 0),
        Preferences.getDouble("vy", 0),
        Preferences.getDouble("omega", 0)));
  }
}