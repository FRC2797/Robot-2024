package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkMax left = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax right = new CANSparkMax(13, MotorType.kBrushless);

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    RelativeEncoder leftEnc = left.getEncoder();
    RelativeEncoder rightEnc = right.getEncoder();

    boolean enabled = false;
    BangBangController bangBang = new BangBangController();
    double setpoint = 0; 

    public Shooter() {
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);

        left.setInverted(false);
        right.setInverted(true);

        leftEnc.setVelocityConversionFactor(Constants.kShooterGearRatio);
        rightEnc.setVelocityConversionFactor(Constants.kShooterGearRatio);

        leftEnc.setPositionConversionFactor(Constants.kShooterGearRatio);
        rightEnc.setPositionConversionFactor(Constants.kShooterGearRatio);

        leftEnc.setPosition(0);
        rightEnc.setPosition(0);

        if (RobotBase.isReal()) {
            tab.addDouble("Rotations", this::getRotations);
        }
        tab.addDouble("RPM", this::getRotationsPerMinute);

        tab.add("2000 RPM", getGoToRPMCommand(2000));
    }

    @Override
    public void periodic() {
        if (enabled) {
            left.set(
                bangBang.calculate(getRotationsPerMinute(), setpoint)
            );
            right.set(
                bangBang.calculate(getRotationsPerMinute(), setpoint)
            );
        }
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        left.set(0);
        right.set(0);
    }

    public Command getGoToRPMCommand(double setpoint) {
        Command goToRPM = new StartEndCommand(
            () -> {
                this.setSetpoint(setpoint);
                this.enable();
            },
            () -> {
                this.setSetpoint(0);
                this.disable();
            },
            this
        );

        return goToRPM;
    }

    public Command getGoToPowerCommand(double power) {
        Command goToPower = new StartEndCommand(
            () -> {
                left.set(power);
                right.set(power);

            },
            () -> {
                left.set(0);
                right.set(0);
            },
            this
        );

        return goToPower;
    }


    public boolean atSetpoint() {
        return getRotationsPerMinute() > setpoint;
    }


    public double getRotationsPerMinute() {
        return (Math.abs(leftEnc.getVelocity()) + Math.abs(rightEnc.getVelocity())) / 2;
    }


    public double getRotations() {
        return (
            Math.abs(leftEnc.getPosition()) + Math.abs(rightEnc.getPosition())) / 2;
    }

}
