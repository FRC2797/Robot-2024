package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final int ENTRY_NOT_FOUND = -9999;

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
    Shuffleboard.getTab("Limelight").addDouble("horizontal offset", this::getHorizontalOffset);
  }

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getHorizontalOffset() {
    NetworkTableEntry horizontalOffset = table.getEntry("tx");
    return horizontalOffset.getDouble(ENTRY_NOT_FOUND);
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
}
