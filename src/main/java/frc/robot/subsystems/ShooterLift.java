package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ShooterLift extends PIDSubsystem {
    CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(10, MotorType.kBrushless);

    RelativeEncoder encoder = motor.getEncoder();
    RelativeEncoder encoder2 = motor2.getEncoder();

    public ShooterLift() {
        super(new PIDController(0.4, 0, 0));
        encoder.setPositionConversionFactor(Constants.kShooterLiftGearRatio);
        encoder2.setPositionConversionFactor(Constants.kShooterLiftGearRatio);
        brakeMotors();
    }   

    // Setpoint is from 0 to 1
    public void setPosition(double setpoint) {
        this.setSetpoint(MathUtil.clamp(setpoint, 0, 1));
    }

    // From 0 to 1
    @Override
    public double getMeasurement() {
        double encoderReadingsAveraged = (encoder.getPosition() + encoder2.getPosition()) / 2;
        return encoderReadingsAveraged / Constants.kShooterLiftMaxRotations;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        motor.set(output);
        motor2.set(output);
    }

    public void brakeMotors(){
        motor.setIdleMode(IdleMode.kBrake);
        motor2.setIdleMode(IdleMode.kBrake);
    }
}
