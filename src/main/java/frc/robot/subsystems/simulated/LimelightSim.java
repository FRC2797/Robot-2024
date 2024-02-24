package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;

public class LimelightSim extends Limelight {
  private final int ENTRY_NOT_FOUND = -9999;

  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

  public double horizontalOffset = 0;
  public double getHorizontalOffset() {
    return horizontalOffset;  
  }

  public double verticalOffset = 0;
  public double getVerticalOffset() {
    return verticalOffset;  
  }

  public boolean hasTarget = false;
  public boolean hasTarget() {
    return hasTarget;  
  }

  Pipeline currentPipeline = Pipeline.placeholder;
  @Override
  protected void switchPipeline(Pipeline pipeline) {
    currentPipeline = pipeline;
  }

  public int seenAprilTag = 1;
  public int getSeenAprilTag() {
    return seenAprilTag;
  }


  Measure<Distance> distance = Inches.of(0);
  public Measure<Distance> getDistance() {
    return distance;
  }
}
