package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);

    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    public Intake() {
        motor.setInverted(false);

        motor.setIdleMode(IdleMode.kCoast);

        tab.add(this.getSetSpeedCommand(0).withName("Go to 0%"));
        tab.add(this.getSetSpeedCommand(0.1).withName("Go to 10%"));
        tab.add(this.getSetSpeedCommand(0.2).withName("Go to 20%"));
        tab.add(this.getSetSpeedCommand(0.3).withName("Go to 30%"));
        tab.add(this.getSetSpeedCommand(0.4).withName("Go to 40%"));
        tab.add(this.getSetSpeedCommand(0.5).withName("Go to 50%"));
        tab.add(this.getSetSpeedCommand(0.6).withName("Go to 60%"));
        tab.add(this.getSetSpeedCommand(0.7).withName("Go to 70%"));
        tab.add(this.getSetSpeedCommand(0.8).withName("Go to 80%"));
        tab.add(this.getSetSpeedCommand(0.9).withName("Go to 90%"));
        tab.add(this.getSetSpeedCommand(1).withName("Go to 100%"));
    }

    public void setMotors(double speed) {
        motor.set(speed);
    }

    public Command getSetSpeedCommand(double speed) {
        return new StartEndCommand(() -> setMotors(speed), () -> setMotors(0), this);
    }
    // RelativeEncoder encoder = motor.getEncoder();
    // BangBangController bangBang = new BangBangController();
    // AnalogInput distanceSensor = new AnalogInput(0);

    // double setpoint = 0;

    // public Intake() {
    //     encoder.setVelocityConversionFactor(Constants.kIntakeGearRatio);
    //     motor.setIdleMode(IdleMode.kCoast);
    // }

    // @Override
    // public void periodic() {
    //     motor.set(
    //         bangBang.calculate(getRotationsPerMinute(), setpoint)
    //     );
    // }

    // public void setSetpoint(double rpm) {
    //     this.setpoint = rpm;
    // }

    // public Command intake(double rpm) {
    //     return new StartEndCommand(() -> this.setSetpoint(rpm), () -> this.setSetpoint(0), this);
    // }

    // public Command intake() {
    //     return intake(Constants.intakeDefaultRPM);
    // }

    // public Command intakeUntilFullyIn() {
    //     double distanceWhenFullyIn = 999;
    //     double tolerance = 1;
    //     return intake(30).until(
    //         () -> MathUtil.isNear(distanceWhenFullyIn, getDistance(), tolerance)
    //     );
    // }

    // public double getDistance() {
    //     return distanceSensor.getValue();
    // }

    // public double getRotationsPerMinute() {
    //     return encoder.getVelocity();
    // }
}
