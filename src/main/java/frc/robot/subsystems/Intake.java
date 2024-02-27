package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
    protected static CANSparkMaxSimWrapper motor = new CANSparkMaxSimWrapper(13, MotorType.kBrushless);
    RelativeEncoder encoder = motor.getEncoder();
    BangBangController bangBang = new BangBangController();
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public Intake() {
        motor.setInverted(false);

        motor.setIdleMode(IdleMode.kCoast);
        tab.addBoolean("Is the ring in?", this::noteIsIn);
        tab.addDouble("Current proximity", this::getProximity);
        tab.addDouble("Current encoder position", encoder::getPosition);

        tab.add(this.getGoToPowerCommand(0).withName("Go to 0%"));
        tab.add(this.getGoToPowerCommand(0.1).withName("Go to 10%"));
        tab.add(this.getGoToPowerCommand(0.2).withName("Go to 20%"));
        tab.add(this.getGoToPowerCommand(0.3).withName("Go to 30%"));
        tab.add(this.getGoToPowerCommand(0.4).withName("Go to 40%"));
        tab.add(this.getGoToPowerCommand(0.5).withName("Go to 50%"));
        tab.add(this.getGoToPowerCommand(0.6).withName("Go to 60%"));
        tab.add(this.getGoToPowerCommand(0.7).withName("Go to 70%"));
        tab.add(this.getGoToPowerCommand(0.8).withName("Go to 80%"));
        tab.add(this.getGoToPowerCommand(0.9).withName("Go to 90%"));
        tab.add(this.getGoToPowerCommand(1).withName("Go to 100%"));

        tab.add("Intake until note is in", intakeUntilNoteIsIn());
    }

    public Command intake(double speed) {
        return new StartEndCommand(
            () -> motor.set(speed),
            () -> motor.set(0),
            this
        );
    }

    public boolean noteIsIn(){
        return getProximity() > 95;
    }

    public double getProximity() {
        return colorSensor.getProximity();
    }

    public Command reverseIntake() {
        return intake(-0.2);
    }

    public Command intakeInitially() {
        return intake(0.2);
    }

    public Command intakeUntilNoteIsIn() {
        return intakeInitially().until(() -> noteIsIn()).andThen(intakeInitially().withTimeout(0.5));
    }

    public Command intakeIntoShooter() {
        return intake(1).withTimeout(1.5);
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
