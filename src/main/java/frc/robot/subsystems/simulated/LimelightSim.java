package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Limelight;

public class LimelightSim extends Limelight {
  private final int ENTRY_NOT_FOUND = -9999;

  ShuffleboardTab tab = Shuffleboard.getTab("Limelight Sim");

  public LimelightSim() {
    tab.addBoolean("Has target", this::hasTarget);
    tab.add(runOnce(() -> setHasTarget(true)).withName("Set to have target"));
    tab.add(runOnce(() -> setHasTarget(false)).withName("Set to not have target"));
    tab.add(runOnce(() -> setBotPose2d_wpiBlue(new Pose2d(2, 3, new Rotation2d()))).withName("Set to have a pose of (2, 3)"));
    tab.add(runOnce(() -> setBotPose2d_wpiBlue(new Pose2d(0, 0, new Rotation2d()))).withName("Set to have a pose of (0, 0)"));
    tab.addString("Current pose", () -> botPose2d_wpiBlue.toString());
  }

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

  public void setHasTarget(boolean hasTarget) {
      this.hasTarget = hasTarget;
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

  public Pose2d botPose2d_wpiBlue = new Pose2d();
  @Override
  public Pose2d getBotPose2d_wpiBlue() {
    return botPose2d_wpiBlue;
  }

  public void setBotPose2d_wpiBlue(Pose2d botPose2d_wpiBlue) {
    this.botPose2d_wpiBlue = botPose2d_wpiBlue;
  }


  Measure<Distance> distance = Inches.of(0);
  public Measure<Distance> getDistance() {
    return distance;
  }
}
