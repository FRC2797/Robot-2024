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
  public enum Pipeline {
    placeholder(0),
    anotherPlaceholder(1);

    public final int value;

    Pipeline(int value) {
      this.value = value;
    }
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

  Pipeline currentPipeline = Pipeline.placeholder;
  private void switchPipeline(Pipeline pipeline) {
    currentPipeline = pipeline;
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

  public int seenAprilTag = 1;
  public int getSeenAprilTag() {
    return seenAprilTag;
  }


  Measure<Distance> distance = Inches.of(0);
  public Measure<Distance> getDistance() {
    return distance;
  }
}
