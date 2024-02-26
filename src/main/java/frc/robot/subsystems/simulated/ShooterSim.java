package frc.robot.subsystems.simulated;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter;

public class ShooterSim extends Shooter {
  private final DCMotor m_armGearbox = DCMotor.getNEO(2);
  private final FlywheelSim leftSim = new FlywheelSim(m_armGearbox, 1, 0.1);
  private final FlywheelSim rightSim = new FlywheelSim(m_armGearbox, 1, 0.1);

  public void simulationPeriodic() {
    leftSim.setInput(left.getAppliedOutput());
    rightSim.setInput(right.getAppliedOutput());

    leftSim.update(0.020);
    rightSim.update(0.020);
  }

  @Override
  public double getLeftRPM() {
    return leftSim.getAngularVelocityRPM();
  }

  @Override
  public double getRightRPM() {
    return rightSim.getAngularVelocityRPM();
  }

  @Override
  public double getRotations() {
    System.out.println("getRotations is NOT SIMULATED");
    return 0;
  }
}
