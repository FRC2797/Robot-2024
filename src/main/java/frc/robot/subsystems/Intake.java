package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(11, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    BangBangController bangBang = new BangBangController();

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

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getRotationsPerMinute() {
        return encoder.getVelocity();
    }
}
