package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;

public class CANSparkMaxSimWrapper extends CANSparkMax {
    public CANSparkMaxSimWrapper(int deviceId, CANSparkLowLevel.MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void set(double speed) {
        double maxVoltage = 12;
        speed = MathUtil.clamp(speed, -1, 1);

        if (RobotBase.isSimulation()) {
            super.setVoltage(speed * maxVoltage);
        } else {
            super.set(speed);
        }
    }
}
