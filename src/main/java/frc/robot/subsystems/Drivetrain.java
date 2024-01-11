package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.showNonessentialShuffleboardInfo;

public class Drivetrain extends SubsystemBase {
  final double WHEEL_DIAMETER = 7 + (3 / 8);
  final double GEAR_RATIO = 1 / 8.45864661654;

  private DifferentialDrive drive;

  private CANSparkMax right;
  private CANSparkMax left;

  private RelativeEncoder rightEnc;
  private RelativeEncoder leftEnc;

  public Drivetrain() {

    configureMotorControllersAndDrivetrain();
    configureEncoders();
    if (showNonessentialShuffleboardInfo)
      setUpShuffleboard();
  }

  private double getWheelRotations() {
    double total = +(rightEnc.getPosition() * -1);

    return total;
  }

  public double getDistanceDrivenInInches() {
    return getWheelRotations() * WHEEL_DIAMETER * Math.PI;
  }

  public void arcadeDrive(double xSpeed, double rotation) {
    final boolean INPUTS_SQUARED = false;
    drive.arcadeDrive(xSpeed, rotation, INPUTS_SQUARED);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    final boolean INPUTS_SQUARED = false;
    drive.tankDrive(leftSpeed, rightSpeed, INPUTS_SQUARED);
  }

  private void setMotorsToBrake() {
    right.setIdleMode(IdleMode.kBrake);
    left.setIdleMode(IdleMode.kBrake);
  }

  private void setMotorsToCoast() {
    right.setIdleMode(IdleMode.kCoast);
    left.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    rightEnc.setPosition(0);
    leftEnc.setPosition(0);
  }

  private void setUpShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    tab.add(this);
    tab.add(drive);
    tab.addDouble("Distance drive in inches", this::getDistanceDrivenInInches);
    tab.addDouble("Get wheel rotations", this::getWheelRotations);
    tab.addDouble("Right Encoder get position", rightEnc::getPosition);
    tab.addDouble("Left Encoder get position", leftEnc::getPosition);
  }

  private void configureMotorControllersAndDrivetrain() {
    final int RIGHT = 1;
    final int LEFT = 2;

    right = new CANSparkMax(RIGHT, kBrushless);
    left = new CANSparkMax(LEFT, kBrushless);

    right.setInverted(true);
    left.setInverted(false);

    drive = new DifferentialDrive(left, right);
    drive.setDeadband(0);
    setMotorsToCoast();
  }

  private void configureEncoders() {
    rightEnc = right.getEncoder();
    leftEnc = left.getEncoder();

    rightEnc.setPositionConversionFactor(GEAR_RATIO);
    leftEnc.setPositionConversionFactor(GEAR_RATIO);
    resetEncoders();
  }
}
