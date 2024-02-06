package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    BangBangController bangBang = new BangBangController();
    AnalogInput distanceSensor = new AnalogInput(0);

    double setpoint = 0;

    public Intake() {
        encoder.setVelocityConversionFactor(Constants.kIntakeGearRatio);
        motor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic() {
        motor.set(
            bangBang.calculate(getRotationsPerMinute(), setpoint)
        );
    }

    public void setSetpoint(double rpm) {
        this.setpoint = rpm;
    }

    public Command intake(double rpm) {
        return new StartEndCommand(() -> this.setSetpoint(rpm), () -> this.setSetpoint(0), this);
    }

    public Command intake() {
        return intake(Constants.intakeDefaultRPM);
    }

    public Command intakeUntilFullyIn() {
        double distanceWhenFullyIn = 999;
        double tolerance = 1;
        return intake(30).until(
            () -> MathUtil.isNear(distanceWhenFullyIn, getDistance(), tolerance)
        );
    }

    public double getDistance() {
        return distanceSensor.getValue();
    }

    public double getRotationsPerMinute() {
        return encoder.getVelocity();
    }
}
