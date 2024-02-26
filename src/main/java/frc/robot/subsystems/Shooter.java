package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    protected CANSparkMaxSimWrapper left = new CANSparkMaxSimWrapper(11, MotorType.kBrushless);
    protected CANSparkMaxSimWrapper right = new CANSparkMaxSimWrapper(12, MotorType.kBrushless);

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    protected RelativeEncoder leftEnc = left.getEncoder();
    protected RelativeEncoder rightEnc = right.getEncoder();

    boolean enabled = false;

    // only kS and kV are necessary
    SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(0, 0);
    PIDController leftController = new PIDController(0.03, 0, 0);

    SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(0, 0);
    PIDController rightController = new PIDController(0.03, 0, 0);

    double setpoint = 0; 

    public Shooter() {
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);

        left.setInverted(true);
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
        tab.add("99999 RPM", getGoToRPMCommand(99999));

        tab.addDouble("Left RPM", this::getLeftRPM);
        tab.addDouble("Right RPM", this::getRightRPM);

    }

    @Override
    public void periodic() {
        if (enabled) {
            left.setVoltage(
                leftController.calculate(getLeftRPM(), setpoint) + leftFeedforward.calculate(setpoint)
            );
            right.setVoltage(
                rightController.calculate(getRightRPM(), setpoint) + rightFeedforward.calculate(setpoint)
            );
        }
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enable() {
        enabled = true;
        leftController.reset();
        rightController.reset();
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
        return (getLeftRPM() > setpoint) && getRightRPM() > setpoint;
    }


    public double getRotationsPerMinute() {
        return (Math.abs(leftEnc.getVelocity()) + Math.abs(rightEnc.getVelocity())) / 2;
    }

    public double getLeftRPM() {
        return leftEnc.getVelocity();
    }

    public double getRightRPM() {
        return rightEnc.getVelocity();
    }


    public double getRotations() {
        return (
            Math.abs(leftEnc.getPosition()) + Math.abs(rightEnc.getPosition())) / 2;
    }

    public Command sysIdQuasistaticForLeft(SysIdRoutine.Direction direction) {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> left.setVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by URCL
                this
            )
        );

        // The methods below return Command objects
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdQuasistaticForRight(SysIdRoutine.Direction direction) {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> right.setVoltage(voltage.in(Volts)),
                null, // No log consumer, since data is recorded by URCL
                this
            )
        );

        // The methods below return Command objects
        return sysIdRoutine.quasistatic(direction);
    }

}
