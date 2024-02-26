package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;

public class ShooterLift extends PIDSubsystem {
    // DONT DIRECTLY SET MOTORS, use the "setMotors" wrapper to ensure the limit switches stop the motors when necessary
    protected CANSparkMaxSimWrapper left = new CANSparkMaxSimWrapper(9, MotorType.kBrushless);
    protected CANSparkMaxSimWrapper right = new CANSparkMaxSimWrapper(10, MotorType.kBrushless);

    protected RelativeEncoder leftEncoder = left.getEncoder();
    protected RelativeEncoder rightEncoder = right.getEncoder();

    private DigitalInput bottomLimitSwitch = new DigitalInput(0);
    private DigitalInput topLimitSwitch = new DigitalInput(1);

    // TODO: this needs to be measured on the actual robot
    static protected final Measure<Angle> atRest = Degrees.of(10);

    ShuffleboardTab tab = Shuffleboard.getTab("ShooterLift");

    public ShooterLift() {
        super(getPidController());

        leftEncoder.setPositionConversionFactor(1 / Constants.kShooterLiftGearRatio);
        rightEncoder.setPositionConversionFactor(1 / Constants.kShooterLiftGearRatio);


        left.setInverted(false);
        right.setInverted(true);

        resetEncoderPositions();

        brakeMotors();
        tab.addDouble("Current measurement", this::getMeasurement);
        tab.addDouble("Current setpoint", this::getSetpoint);
        tab.addDouble("Current left speed", left::get);
        tab.addDouble("Current right speed", right::get);

        tab.addDouble("left encoder value in rotations", leftEncoder::getPosition);
        tab.addDouble("right encoder value in rotations", rightEncoder::getPosition);

        tab.addBoolean("Is fully up", this::isFullyUp);
        tab.addBoolean("Is fully down", this::isFullyDown);

        ShuffleboardLayout goToPosition = tab.getLayout("Go to position", BuiltInLayouts.kList).withSize(2, 2).withProperties(Map.of("Label position", "HIDDEN"));
        for (double i = 0; i <= 90; i += 5) {
            goToPosition.add(
                getGoToPositionCommand(i).withName(String.format("Go to %.2f degrees", i))
            );
        }

        ShuffleboardLayout goToPowerList = tab.getLayout("Go to power", BuiltInLayouts.kList).withSize(2, 2).withProperties(Map.of("Label position", "HIDDEN"));
        for (double i = -1; i < 1.05; i += 0.05) {
            goToPowerList.add(
                getGoToPowerCommand(i).withName(String.format("Go to %.2f power", i))
            );
        }

        tab.add("Reset Encoders", runOnce(this::resetEncoderPositions));

        this.disable();
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean leftGoingUp = left.get() > 0;
        boolean leftGoingDown = left.get() < 0;

        boolean rightGoingUp = right.get() > 0;
        boolean rightGoingDown = right.get() < 0;

        if (isFullyDown() && (leftGoingDown || rightGoingDown)) {
            setMotors(0);
        }

        if (isFullyUp() && (leftGoingUp || rightGoingUp)) {
            setMotors(0);
        }

        if (isFullyDown()) {
            resetEncoderPositions();
        }

        if (isFullyUp()) {
            leftEncoder.setPosition(Radians.of(Math.PI / 2).in(Rotations));
            rightEncoder.setPosition(Radians.of(Math.PI / 2).in(Rotations));
        }
    }

    public void setPosition(Measure<Angle> setpoint) {
        this.setSetpoint(MathUtil.clamp(setpoint.in(Radians), atRest.in(Radians), Math.PI / 2));
    }

    public Measure<Angle> getMeasurementAsMeasure() {
        Measure<Angle> encoderReading = rightGetPosition();
        return encoderReading;
    }

    @Override
    public double getMeasurement() {
        return getMeasurementAsMeasure().in(Radians);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        setMotors(output);
    }

    public void brakeMotors(){
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
    }

    public Command getGoToPositionCommand(Measure<Angle> setpoint) {
        Command goToPosition = new StartEndCommand(
            () -> {
                this.setPosition(setpoint);
                this.enable();
            },
            () -> {
                this.setPosition(Radians.of(0));
                this.disable();
            },
            this
        );

        return goToPosition;
    }

    public Command getGoToPositionCommand(double degrees) {
        return getGoToPositionCommand(Degrees.of(degrees));
    }

    public Command getGoToPowerCommand(double power) {
        Command goToPower = run(() -> {
            setMotors(getMeasurement() > 0.5 ? 0 : power);
        }).finallyDo(() -> {
            setMotors(0);
        });

        return goToPower;
    }


    public void unbrakeMotors(){
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);
    }

    private static PIDController getPidController() {
        PIDController pid = new PIDController(0.45, 0.13, 0);
        pid.setTolerance(0.05);

        pid.enableContinuousInput(-Math.PI, Math.PI);
        return pid;
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getSetpoint(), getMeasurement(), 0.05);
    }

    private void setMotors(double speed) {
        if ((isFullyDown() && speed < 0) || (isFullyUp() && speed > 0)) {
            left.set(0);
            right.set(0);
        } else {
            left.set(speed);
            right.set(speed);
        }
    }

    protected boolean isFullyDown() {
        return !bottomLimitSwitch.get();
    }

    protected boolean isFullyUp() {
        return !topLimitSwitch.get();
    }

    Measure<Angle> rightGetPosition() {
        return Rotations.of(rightEncoder.getPosition());
    }

    Measure<Angle> leftGetPosition() {
        return Rotations.of(leftEncoder.getPosition());
    }


    public void resetEncoderPositions() {
        leftEncoder.setPosition(atRest.in(Rotations));
        rightEncoder.setPosition(atRest.in(Rotations));
    }
}
