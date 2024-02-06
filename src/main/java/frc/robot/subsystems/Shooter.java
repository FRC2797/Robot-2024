package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkMax left = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax right = new CANSparkMax(13, MotorType.kBrushless);

    RelativeEncoder leftEnc = left.getEncoder();
    RelativeEncoder rightEnc = right.getEncoder();

    BangBangController bangBang = new BangBangController();
    double setpoint = 0; 

    public Shooter() {
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);

        leftEnc.setVelocityConversionFactor(Constants.kShooterGearRatio);
        rightEnc.setVelocityConversionFactor(Constants.kShooterGearRatio);
    }

    @Override
    public void periodic() {
        left.set(
            bangBang.calculate(getRotationsPerMinute(), setpoint)
        );
        left.set(
            bangBang.calculate(getRotationsPerMinute(), setpoint)
        );
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getRotationsPerMinute() {
        return (leftEnc.getVelocity() + rightEnc.getVelocity()) / 2;
    }
}
