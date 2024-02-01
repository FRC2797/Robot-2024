package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterLift extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(10, MotorType.kBrushless);

    public ShooterLift() {
        
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
