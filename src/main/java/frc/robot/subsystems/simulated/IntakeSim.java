package frc.robot.subsystems.simulated;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;

public class IntakeSim extends Intake {

  private final DCMotor m_armGearbox = DCMotor.getNeo550(1);

  private final FlywheelSim intake = new FlywheelSim(m_armGearbox, 5, 0.00017548352);

  ShuffleboardTab tab = Shuffleboard.getTab("Intake Simulated");

  double minProximityValue = 0;
  double proximitySensorDistanceToNote = minProximityValue;

  public IntakeSim() {
    tab.addDouble("RPM", intake::getAngularVelocityRPM);
    tab.addDouble("Distance to note", () -> proximitySensorDistanceToNote);
  }

  public void simulationPeriodic() {
    intake.setInput(motor.getAppliedOutput());

    intake.update(0.020);

    double distanceToNoteSimSpeed = 5;
    if (MathUtil.isNear(0, intake.getAngularVelocityRPM(), 1)) {

    } else if (intake.getAngularVelocityRPM() > 0) {
        proximitySensorDistanceToNote += distanceToNoteSimSpeed;
    } else if (intake.getAngularVelocityRPM() < 0) {
        proximitySensorDistanceToNote = Math.min(proximitySensorDistanceToNote - distanceToNoteSimSpeed, minProximityValue);
    }
  }

  public void loadNote() {
    proximitySensorDistanceToNote = minProximityValue;
  }

  @Override
  public double getProximity() {
      return proximitySensorDistanceToNote;
  }
}
