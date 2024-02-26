// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  RobotContainer robotContainer = new RobotContainer();
  @Override
  public void robotInit() {
      boolean dataLoggingEnabled = false;
      if (dataLoggingEnabled) {
        DataLogManager.start();
        URCL.start();
      }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    Command auto = robotContainer.getAutonomousCommand();
    if (auto == null) {
      System.out.println("No Auto command selected");
    } else {
        robotContainer.getAutonomousCommand().schedule();
    }
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
