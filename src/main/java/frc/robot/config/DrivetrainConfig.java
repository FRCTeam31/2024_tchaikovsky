package frc.robot.config;

import edu.wpi.first.math.util.Units;
import prime.control.PrimePIDConstants;

public class DrivetrainConfig {

  // Physical properties
  public double TrackWidthMeters;
  public double WheelBaseMeters;
  public double WheelBaseCircumferenceMeters;
  public double MaxSpeedMetersPerSecond;
  public double MaxAccelerationMetersPerSecondSquared;
  public double MaxAngularSpeedRadians;

  // CAN IDs
  public int PigeonId;

  // Control properties
  public double DriveDeadband;
  public double DeadbandCurveWeight;

  // PID configs
  public PrimePIDConstants DrivePID;
  public PrimePIDConstants SteeringPID;
  public PrimePIDConstants SnapToPID;
  public PrimePIDConstants PathingTranslationPid;
  public PrimePIDConstants PathingRotationPid;

  // Limelight configs
  public String LimelightRearName;
  public String LimelightFrontName;

  /**
   * Gets a default instance of a DrivetrainConfig with all properties set to 2024 robot values
   */
  public DrivetrainConfig() {
    TrackWidthMeters = 0.51181;
    WheelBaseMeters = 0.67945;
    WheelBaseCircumferenceMeters = Math.PI * 0.7778174593052;
    MaxSpeedMetersPerSecond = Units.feetToMeters(20);
    MaxAccelerationMetersPerSecondSquared = Units.feetToMeters(15);
    MaxAngularSpeedRadians = Math.PI * 3;
    PigeonId = 1;
    DriveDeadband = 0.15;
    DeadbandCurveWeight = 0.5;
    DrivePID = new PrimePIDConstants(0.019, 0, 0, 0, 0.091, 0, 0.05);
    SteeringPID = new PrimePIDConstants(2, 0, 0);
    SnapToPID = new PrimePIDConstants(6, 0, 0);
    PathingTranslationPid = new PrimePIDConstants(3, 0, 0);
    PathingRotationPid = new PrimePIDConstants(2, 0, 0);
    LimelightRearName = "limelight-rear";
    LimelightFrontName = "limelight-front";
  }
}
