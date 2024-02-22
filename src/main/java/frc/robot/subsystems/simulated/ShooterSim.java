package frc.robot.subsystems.simulated;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class ShooterSim extends Shooter {
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);
  private final FlywheelSim shooterSim = new FlywheelSim(m_armGearbox, 1, 0.1);

  double minProximityValue = 0;
  double proximitySensorDistanceToNote = minProximityValue;

  public void simulationPeriodic() {
    shooterSim.setInput((left.getAppliedOutput() + right.getAppliedOutput()) / 2);

    shooterSim.update(0.020);

    SmartDashboard.putNumber("Shooter wheel rpm sim", shooterSim.getAngularVelocityRPM());
  }

  @Override
  public double getRotationsPerMinute() {
    return shooterSim.getAngularVelocityRPM();
  }

  @Override
  public double getRotations() {
    System.out.println("getRotations is NOT SIMULATED");
    return 0;
  }
}
