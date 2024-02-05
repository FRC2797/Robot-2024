package frc.robot.subsystems;

import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterLift extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(10, MotorType.kBrushless);

    RelativeEncoder encoder = motor.getEncoder();
    RelativeEncoder encoder2 = motor2.getEncoder();

    PIDController pidController = new PIDController(0.4, 0, 0);
    double setpoint = 0;


    public ShooterLift() {
        encoder.setPositionConversionFactor(Constants.kShooterLiftGearRatio);
        encoder2.setPositionConversionFactor(Constants.kShooterLiftGearRatio);
    }   

    @Override
    public void periodic() {
        double output = pidController.calculate(getPosition(), setpoint);
        motor.set(output);
        motor2.set(output);
    }

    // Setpoint is from 0 to 1
    public void setPosition(double setpoint) {
        this.setpoint = MathUtil.clamp(setpoint, 0, 1);
    }

    // From 0 to 1
    public double getPosition() {
        double encoderReadingsAveraged = (encoder.getPosition() + encoder2.getPosition()) / 2;
        return encoderReadingsAveraged / Constants.kShooterLiftMaxRotations;
    }

    public void enableMotors(boolean on){
        if (on){
            motor.setIdleMode(IdleMode.kBrake);
            motor2.setIdleMode(IdleMode.kBrake);
        }
    }

    public void disableMotors(boolean off){
        if (off){
            motor.setIdleMode(IdleMode.kCoast);
            motor2.setIdleMode(IdleMode.kCoast);
        }
    }

    public void shooterUp(double speed) {
        motor.set(speed);
        motor2.set(speed);
    }

    public void shooterDown(double speed) {
        motor.set(-speed);
        motor2.set(-speed);
    }

    public void shooterStop() {
        motor.set(0);
        motor2.set(0);
    }
}
