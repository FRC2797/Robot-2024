package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.InterpolatingTreeMap;

public class Constants {
  public static boolean showNonessentialShuffleboardInfo = true;
  public static double kShooterLiftGearRatio = 50.8; 
  public static double kShooterLiftLengthInches = 31;
  public static double kShooterLiftMassRoughlyPounds = 25;
  public static double kShooterLiftMaxAngleDegrees = 130;
  public static double kShooterLiftMinAngleDegrees = -85;
  public static double kShooterLiftMaxRotationsToFullyVertical = 0.18;

  public static double kIntakeGearRatio = 1/999999;
  public static double kShooterGearRatio = 1;

  public static final class Limelight {
    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double mountingAngleDegrees = 10;

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 20.0;

    // distance from the target to the floor
    public static double goalHeightInches = 60.0; 

    
  }

  public static final class ModuleConstants {
    // Need to update wheelDiameter
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = 5;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static final class speakerInterpolationConstants {
    public static InterpolatingDoubleTreeMap getShooterLeft() {
      InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
      return map;
    }

    public static InterpolatingDoubleTreeMap getShooterRight() {
      InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
      return map;
    }

    public static InterpolatingDoubleTreeMap getLiftHeight() {
      InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
      return map;
    }
  }

  public static final class OIConstants {
    public static final double kDeadband = 0.05;
  }

}