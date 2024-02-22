package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    protected static CANSparkMaxSimWrapper motor = new CANSparkMaxSimWrapper(11, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    BangBangController bangBang = new BangBangController();
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public Intake() {
        motor.setInverted(false);

        motor.setIdleMode(IdleMode.kCoast);

        tab.addBoolean("Is the ring in?", this::noteIsIn);
    }

    public Command intake(double speed) {
        return new StartEndCommand(
            () -> motor.set(speed),
            () -> motor.set(0),
            this
        );
    }


    public static final double kIntakePower = 0.77;
    public Command intake() {
        return intake(kIntakePower);
    }

    public Command reverseIntake() {
        return intake(-kIntakePower);
    }


    public boolean noteIsIn(){
        return getProximity() > 150;
    }

    public double getProximity() {
        return colorSensor.getProximity();
    }

    public Command intakeUntilNoteIsIn() {
        return intake().until(() -> noteIsIn());
    }

    public Command intakeIntoShooter() {
        return intake().withTimeout(3);
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
