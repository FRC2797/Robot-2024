package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.showNonessentialShuffleboardInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Limelight {
  private final int ENTRY_NOT_FOUND = -9999;

  ShuffleboardTab tab = Shuffleboard.getTab("Limelight");
  public enum Pipeline {
    placeholder(0),
    anotherPlaceholder(1);

    public final int value;

    Pipeline(int value) {
      this.value = value;
    }
  }

  public Limelight() {
    Shuffleboard.getTab("Driver").addBoolean("Has Target", this::hasTarget);
    if (showNonessentialShuffleboardInfo) {
        tab.addDouble("horizontal offset", this::getHorizontalOffset);
        tab.addBoolean("Has Target", this::hasTarget);
    }
  }

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getHorizontalOffset() {
    NetworkTableEntry horizontalOffset = table.getEntry("tx");
    return horizontalOffset.getDouble(ENTRY_NOT_FOUND);
  }

  public double getVerticalOffset() {
    NetworkTableEntry verticalOffset = table.getEntry("ty");
    return verticalOffset.getDouble(ENTRY_NOT_FOUND);
  }

  public boolean hasTarget() {
    NetworkTableEntry hasTarget = table.getEntry("tv");
    return hasTarget.getInteger(0) == 1 ? true : false;
  }

  private void switchPipeline(Pipeline pipeline) {
    table.getEntry("pipeline").setNumber(pipeline.value);
  }

  public Command switchPipelineCommand(Pipeline pipeline) {
    return runOnce(() -> switchPipeline(pipeline)).andThen(waitSeconds(0.1));
  }

  public Measure<Distance> getDistance() {
    double angleToGoalDegrees = Constants.Limelight.mountingAngleDegrees + getVerticalOffset();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (Constants.Limelight.goalHeightInches - Constants.Limelight.limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  
    return Inches.of(distanceFromLimelightToGoalInches);
  }
}
