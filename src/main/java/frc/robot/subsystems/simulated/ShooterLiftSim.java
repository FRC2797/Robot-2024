package frc.robot.subsystems.simulated;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterLift;
public class ShooterLiftSim extends ShooterLift {
  private boolean isFullyUp = false;
  private boolean isFullyDown = false;

  // Constants taken from the arm sim example
  public final SingleJointedArmSim armSim = 
    new SingleJointedArmSim(
      DCMotor.getNEO(2),
      Constants.kShooterLiftGearRatio,
      SingleJointedArmSim.estimateMOI(Units.inchesToMeters(Constants.kShooterLiftLengthInches), Units.lbsToKilograms(Constants.kShooterLiftMassRoughlyPounds)),
      Units.inchesToMeters(Constants.kShooterLiftLengthInches),
      ShooterLift.atRest.in(Radians),
      Math.PI / 2,
      true,
      0,
      VecBuilder.fill(2.0 * Math.PI / 4096) // Add noise with a std-dev of 1 tick
    );


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

  private ShuffleboardTab tab = Shuffleboard.getTab("Shooter Lift Simulated");

  public ShooterLiftSim() {
    tab.add("ShooterLift sim visualization", m_mech2d);
    
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    tab.add(Commands.runOnce(() -> setFullyUp(true)).withName("set fullyUp to true"));
    tab.add(Commands.runOnce(() -> setFullyDown(true)).withName("set fullyDown to true"));

    tab.add(Commands.runOnce(() -> setFullyUp(false)).withName("set fullyUp to false"));
    tab.add(Commands.runOnce(() -> setFullyDown(false)).withName("set fullyDown to false"));
  }

  @Override
  public void simulationPeriodic() {
      // just average them together
      armSim.setInput((left.getAppliedOutput() + right.getAppliedOutput()) / 2);

      armSim.update(0.020);

      leftEncoder.setPosition(Units.radiansToRotations(armSim.getAngleRads()));
      rightEncoder.setPosition(Units.radiansToRotations(armSim.getAngleRads()));
      m_arm.setAngle(radiansToDegrees(armSim.getAngleRads()));
  }

  @Override
  public boolean isFullyUp() {
    return isFullyUp;
  }

  public void setFullyUp(boolean isFullyUp) {
    this.isFullyUp = isFullyUp;
  }

  @Override
  public boolean isFullyDown() {
    return isFullyDown;
  }

  public void setFullyDown(boolean isFullyDown) {
    this.isFullyDown = isFullyDown;
  }

  @Override
  public void setInitialMeasurement(double degrees) {
    super.setInitialMeasurement(degrees);
    armSim.setState(degreesToRadians(90), 0);
  }
}
