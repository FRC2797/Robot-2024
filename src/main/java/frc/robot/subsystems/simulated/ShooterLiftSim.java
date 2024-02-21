package frc.robot.subsystems.simulated;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterLift;

public class ShooterLiftSim extends ShooterLift {
  private double measurement = 0;

  // Constants taken from the arm sim example
  private final SingleJointedArmSim armSim = 
    new SingleJointedArmSim(
      DCMotor.getNEO(2),
      Constants.kShooterLiftGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.kShooterLiftLengthInches), Units.lbsToKilograms(Constants.kShooterLiftMassRoughlyPounds)),
      Units.inchesToMeters(Constants.kShooterLiftLengthInches),
      degreesToRadians(Constants.kShooterLiftMinAngleDegrees),
      degreesToRadians(Constants.kShooterLiftMaxAngleDegrees),
      true,
      0,
      VecBuilder.fill(2.0 * Math.PI / 4096) // Add noise with a std-dev of 1 tick
    );


  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Subsystem constructor. */
  public ShooterLiftSim() {
    SmartDashboard.putData("ShooterLift sim visualization", m_mech2d);
    
    m_armTower.setColor(new Color8Bit(Color.kBlue));
  }

  @Override
  public void simulationPeriodic() {
      SmartDashboard.putNumber("Left speed is ", left.get());
      SmartDashboard.putNumber("Right speed is ", right.get());

      // just average them together
      armSim.setInput((left.getAppliedOutput() + right.getAppliedOutput()) / 2);

      // Next, we update it. The standard loop time is 20ms.
      armSim.update(0.020);

      // Finally, we set our simulated encoder's readings and simulated battery voltage
      // convert the encoders to percentages
      double armSimAngleAsPercentage = (armSim.getAngleRads() - degreesToRadians(Constants.kShooterLiftMinAngleDegrees)) / degreesToRadians(Constants.kShooterLiftMaxAngleDegrees);
      measurement = armSimAngleAsPercentage;
      // Update the Mechanism Arm angle based on the simulated arm angle
      m_arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  @Override
  public double getMeasurement() {
      return measurement;
  }
}
