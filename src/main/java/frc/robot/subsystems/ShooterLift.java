package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;

public class ShooterLift extends PIDSubsystem {
    CANSparkMax left = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax right = new CANSparkMax(10, MotorType.kBrushless);

    RelativeEncoder leftEncoder = left.getEncoder();
    RelativeEncoder rightEncoder = right.getEncoder();

    PIDController pid = new PIDController(1.3, 0.1, 0);

    ShuffleboardTab tab = Shuffleboard.getTab("ShooterLift");

    // Should probably have a measurement using radians instead of 0 to 1 with 1 being perfectly vertical
    public ShooterLift() {
        super(getPidController());

        leftEncoder.setPositionConversionFactor(1 / Constants.kShooterLiftGearRatio);
        rightEncoder.setPositionConversionFactor(1 / Constants.kShooterLiftGearRatio);


        left.setInverted(false);
        right.setInverted(true);

        leftEncoder.setPosition(0);
        leftEncoder.setPosition(0);

        brakeMotors();
        tab.addDouble("Current measurement", this::getMeasurement);
        tab.addDouble("Current setpoint", this::getSetpoint);
        tab.addDouble("Current left speed", left::get);
        tab.addDouble("Current right speed", right::get);

        tab.addDouble("left encoder value", leftEncoder::getPosition);
        tab.addDouble("right encoder value", rightEncoder::getPosition);

        tab.add(this.getGoToPositionCommand(0).withName("Go to 0%"));
        tab.add(this.getGoToPositionCommand(0.1).withName("Go to 10%"));
        tab.add(this.getGoToPositionCommand(0.2).withName("Go to 20%"));
        tab.add(this.getGoToPositionCommand(0.3).withName("Go to 30%"));
        tab.add(this.getGoToPositionCommand(0.4).withName("Go to 40%"));
        tab.add(this.getGoToPositionCommand(0.5).withName("Go to 50%"));
        tab.add(this.getGoToPositionCommand(0.6).withName("Go to 60%"));
        tab.add(this.getGoToPositionCommand(0.7).withName("Go to 70%"));
        tab.add(this.getGoToPositionCommand(0.8).withName("Go to 80%"));
        tab.add(this.getGoToPositionCommand(0.9).withName("Go to 90%"));
        tab.add(this.getGoToPositionCommand(1).withName("Go to 100%"));

        tab.add(getGoToPowerCommand(1).withName("Go to full power up"));
        tab.add(getGoToPowerCommand(0.3).withName("Go to 0.3 Power"));
        tab.add(getGoToPowerCommand(-0.3).withName("Go to -0.3 Power"));
        tab.add(getGoToPowerCommand(-1).withName("Go to full power down"));

        this.disable();
    }

    // Setpoint is from 0 to 1
    public void setPosition(double setpoint) {
        this.setSetpoint(MathUtil.clamp(setpoint, 0, 1));
    }

    // From 0 to 1
    @Override
    public double getMeasurement() {
        double encoderReadingsAveraged = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        return encoderReadingsAveraged / Constants.kShooterLiftMaxRotations;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        left.set(output);
        right.set(output);
    }

    public void brakeMotors(){
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
    }

    public Command getGoToPositionCommand(double setpoint) {
        Command goToPosition = new StartEndCommand(
            () -> {
                this.setPosition(setpoint);
                this.enable();
            },
            () -> {
                this.setPosition(0);
                this.disable();
            },
            this
        );

        return goToPosition;
    }

    public Command getGoToPowerCommand(double power) {
        Command goToPower = new StartEndCommand(
            () -> {
                left.set(getMeasurement() > 0.8 ? 0 : power);
                right.set(getMeasurement() > 0.8 ? 0 : power);
            },
            () -> {
                left.set(0);
                right.set(0);
            },
            this
        );

        return goToPower;
    }


    public void unbrakeMotors(){
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);
    }

    private static PIDController getPidController() {
        PIDController pid = new PIDController(0.7, 0.2, 0);
        pid.setTolerance(0.05);
        return pid;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getSetpoint(), getMeasurement(), 0.05);
    }
}
