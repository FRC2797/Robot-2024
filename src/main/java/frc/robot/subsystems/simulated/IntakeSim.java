package frc.robot.subsystems.simulated;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;

public class IntakeSim extends Intake {

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getNeo550(1);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final FlywheelSim intake = new FlywheelSim(m_armGearbox, 5, 0.00017548352);

  double minProximityValue = 0;
  double proximitySensorDistanceToNote = minProximityValue;

  /** Subsystem constructor. */
  public IntakeSim() {
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    intake.setInput(motor.getAppliedOutput());

    // Next, we update it. The standard loop time is 20ms.
    intake.update(0.020);

    double distanceToNoteSimSpeed = 5;
    if (MathUtil.isNear(0, intake.getAngularVelocityRPM(), 1)) {

    } else if (intake.getAngularVelocityRPM() > 0) {
        proximitySensorDistanceToNote += distanceToNoteSimSpeed;
    } else if (intake.getAngularVelocityRPM() < 0) {
        proximitySensorDistanceToNote = Math.min(proximitySensorDistanceToNote - distanceToNoteSimSpeed, minProximityValue);
    }

    SmartDashboard.putNumber("Intake wheel rpm sim", intake.getAngularVelocityRPM());
    SmartDashboard.putNumber("Distance to note", proximitySensorDistanceToNote);
  }

  public void loadNote() {
    proximitySensorDistanceToNote = minProximityValue;
  }

  @Override
  public double getProximity() {
      return proximitySensorDistanceToNote;
  }
}
