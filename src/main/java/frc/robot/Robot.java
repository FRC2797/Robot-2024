// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  CANSparkMax right = new CANSparkMax(10, MotorType.kBrushless);
  // CANSparkMax left = new CANSparkMax(9, MotorType.kBrushless);

  @Override
  public void testInit() {
      right.set(0.25);
      // left.set(0.25);
  }

  @Override
  public void testExit() {
      right.set(0);
      // left.set(0);
  }
  // RobotContainer robotContainer = new RobotContainer();
  // @Override
  // public void robotInit() {
  // }

  // @Override
  // public void robotPeriodic() {
  //   CommandScheduler.getInstance().run();
  // }

  // @Override
  // public void autonomousInit() {
  //   Command auto = robotContainer.getAutonomousCommand();
  //   if (auto == null) {
  //     System.out.println("No Auto command selected");
  //   } else {
  //       robotContainer.getAutonomousCommand().schedule();
  //   }
  // }

  // @Override
  // public void autonomousExit() {
  //   CommandScheduler.getInstance().cancelAll();
  // }
}
