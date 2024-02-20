package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(14, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();

    public Winch() {
        motor.setInverted(false);
    }

    public Command getGoToPowerCommand(double power) {
        Command goToPower = new StartEndCommand(
            () -> {
                motor.set(power);

            },
            () -> {
                motor.set(0);
            },
            this
        );

        return goToPower;
    }

}
