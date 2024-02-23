package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Constants.showNonessentialShuffleboardInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public String getSeenAprilTagAsString() {
    switch (getSeenAprilTag()) {
      case ENTRY_NOT_FOUND:
        return "No Apriltag";

      case 1:
        return "Blue's right source";

      case 2:
        return "Blue's left source";

      case 3:
        return "Red's right subwoofer";

      case 4:
        return "Red's middle subwoofer";

      case 5:
        return "Red's Amp";

      case 6:
        return "Blue's Amp";

      case 7:
        return "Blue's middle subwoofer";

      case 8:
        return "Blue's left subwoofer";

      case 9:
        return "Red's right source";

      case 10:
        return "Red's left source";

      case 11, 12, 13:
        return "Red stage";

      case 14, 15, 16:
        return "Blue stage";
    
      default:
        return "April tag pipeline broken";
    }
  }

  public int getSeenAprilTag() {
    return (int) table.getEntry("tid").getInteger(ENTRY_NOT_FOUND);
  }


  public Pose2d getRobotPoseInFieldSpaceWithBlueOrigin() {
    double[] botpose_wpiblue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    // TODO: Need to figure out what unit this translation2d is in
    Translation2d translation = new Translation2d(botpose_wpiblue[0], botpose_wpiblue[1]);
    Rotation2d rotation = Rotation2d.fromDegrees(botpose_wpiblue[6]);

    return new Pose2d(translation, rotation);
  }

  public Measure<Distance> getDistance() {
    double angleToGoalDegrees = Constants.Limelight.mountingAngleDegrees + getVerticalOffset();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (Constants.Limelight.goalHeightInches - Constants.Limelight.limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  
    return Inches.of(distanceFromLimelightToGoalInches);
  }
}
