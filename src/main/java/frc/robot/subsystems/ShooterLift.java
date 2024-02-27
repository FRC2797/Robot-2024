package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterLift extends ProfiledPIDSubsystem {
    // DONT DIRECTLY SET MOTORS, use the "setMotors" wrapper to ensure the limit switches stop the motors when necessary
    protected CANSparkMaxSimWrapper left = new CANSparkMaxSimWrapper(9, MotorType.kBrushless);
    protected CANSparkMaxSimWrapper right = new CANSparkMaxSimWrapper(10, MotorType.kBrushless);

    protected RelativeEncoder leftEncoder = left.getEncoder();
    protected RelativeEncoder rightEncoder = right.getEncoder();

    private DigitalInput bottomLimitSwitch = new DigitalInput(0);
    private DigitalInput topLimitSwitch = new DigitalInput(1);

    static protected final Measure<Angle> atRest = Degrees.of(2);
    static protected final Measure<Angle> hittingTopLimitSwitch = Degrees.of(130);


    ShuffleboardTab tab = Shuffleboard.getTab("ShooterLift");

    private Measure<Voltage> kS = Volts.of(0);
    private Measure<Voltage> kG = Volts.of(0);

    private Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond.of(0);
    private Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA = VoltsPerRadianPerSecondSquared.of(0);

    // kA can be ommitted, apparently
    ArmFeedforward feedforward = new ArmFeedforward(kS.in(Volts), kG.in(Volts), kV.in(VoltsPerRadianPerSecond), kA.in(VoltsPerRadianPerSecondSquared));

    //static because something was up with how java handles superclasses. Shouldn't be done this way but throws error otherwise
    private static ProfiledPIDController pidController = getPidController();
    private static ProfiledPIDController getPidController() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(RadiansPerSecond.of(0.25), RadiansPerSecond.per(Second).of(0.25));
        ProfiledPIDController pid = new ProfiledPIDController(18, 0, 0, constraints);
        pid.setTolerance(Degrees.of(4).in(Radians));

        pid.enableContinuousInput(-Math.PI, Math.PI);
        return pid;
    }

    public ShooterLift() {
        super(pidController);

        leftEncoder.setPositionConversionFactor(1 / Constants.kShooterLiftGearRatio);
        rightEncoder.setPositionConversionFactor(1 / Constants.kShooterLiftGearRatio);


        left.setInverted(false);
        right.setInverted(true);

        resetEncoderPositions();

        brakeMotors();
        tab.addDouble("Current measurement", () -> getMeasurementAsMeasure().in(Degrees));
        tab.addDouble("Current setpoint", () -> radiansToDegrees(pidController.getSetpoint().position));
        tab.addBoolean("At goal?", this::atGoal);

        tab.addDouble("Current left speed", left::get);
        tab.addDouble("Current right speed", right::get);

        tab.addDouble("left encoder value in rotations", leftEncoder::getPosition);
        tab.addDouble("right encoder value in rotations", rightEncoder::getPosition);

        tab.addBoolean("Is fully up", this::isFullyUp);
        tab.addBoolean("Is fully down", this::isFullyDown);

        tab.add("Reset Encoders", runOnce(this::resetEncoderPositions));

        tab.add("shooter lift go to 30", getGoToPositionCommand(30));
        tab.add("shooter lift go to 45", getGoToPositionCommand(45));
        tab.add("shooter lift go to 70", getGoToPositionCommand(70));
        tab.add("shooter lift go to 80", getGoToPositionCommand(80));
        tab.add("shooter lift go to 90", getGoToPositionCommand(90));
        tab.add("shooter lift go to rest", getGoToPositionCommand(atRest.in(Degrees)));

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
            setMotors(Volts.of(0));
        }

        if (isFullyUp() && (leftGoingUp || rightGoingUp)) {
            setMotors(Volts.of(0));
        }

        if (isFullyDown()) {
            resetEncoderPositions();
        }

        if (isFullyUp()) {
            leftEncoder.setPosition(hittingTopLimitSwitch.in(Rotations));
            rightEncoder.setPosition(hittingTopLimitSwitch.in(Rotations));
        }
    }

    public void setPosition(Measure<Angle> setpoint) {
        this.setGoal((MathUtil.clamp(setpoint.in(Radians), atRest.in(Radians), Math.PI / 2)));
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
    protected void useOutput(double output, TrapezoidProfile.State state) {
        setMotors(Volts.of(output).plus(Volts.of(feedforward.calculate(state.position, state.velocity))));
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

    public Command getGoToPowerCommand(Measure<Voltage> power) {
        Command goToPower = run(() -> {
            setMotors(getMeasurement() > 0.5 ? Volts.of(0.0) : power);
        }).finallyDo(() -> {
            setMotors(Volts.of(0));
        });

        return goToPower;
    }


    public void unbrakeMotors(){
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);
    }


    public boolean atGoal() {
        // don't set tolerance here, set tolerance in pid controller itself
        return pidController.atGoal();
    }

    private void setMotors(Measure<Voltage> speed) {
        if ((isFullyDown() && speed.in(Volts) < 0) || (isFullyUp() && speed.in(Volts) > 0)) {
            left.setVoltage(0);
            right.setVoltage(0);
        } else {
            left.setVoltage(speed.in(Volts));
            right.setVoltage(speed.in(Volts));
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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.setMotors(voltage),
                null, // No log consumer, since data is recorded by URCL
                this
            )
        );

        // The methods below return Command objects
        return sysIdRoutine.quasistatic(direction);
    }
}
