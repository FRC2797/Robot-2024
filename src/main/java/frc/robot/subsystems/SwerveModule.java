package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.showNonessentialShuffleboardInfo;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;

// Swerve code from 0 to autonomous series: https://www.youtube.com/watch?v=0Xi9yb1IMyA&t=1s
// TODO: Need to add the absolute encoder to set the relative turning encoders at startup
public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    ShuffleboardTab tab = Shuffleboard.getTab("Swerve Drive Motors");

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            boolean turningMotorReversed) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        if (showNonessentialShuffleboardInfo) {
            String moduleName = "D: " + driveMotorId + " T: " + turningMotorId;
            tab.addDouble(moduleName + " Angle in degrees", () -> getState().angle.getDegrees());
            tab.addDouble(moduleName + " Drive Position", this::getDrivePosition);
            tab.addDouble(moduleName + " Turning Position", this::getTurningPosition);
            tab.addDouble(moduleName + " Drive Velocity", this::getDriveVelocity);
            tab.addDouble(moduleName + "Turning Velocity", this::getTurningVelocity);
        }
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        // TODO: This will be set the absolute value encoder once we have it
        turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, boolean ignoreLowSpeeds) {
        if (ignoreLowSpeeds && Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        double pidCalculation = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        turningMotor.set(pidCalculation);
    }

    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}