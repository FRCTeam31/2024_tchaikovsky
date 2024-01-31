package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConfig {

  public String ModuleName = "DefaultModuleName";

  public int DriveMotorCanId = 0;
  public int SteeringMotorCanId = 0;
  public int CANCoderCanId = 0;

  public double StartingOffset = 0;
  public boolean DriveInverted = false;
  public boolean SteerInverted = false;

  public PIDConstants SteeringPidConstants = new PIDConstants(0, 0, 0);

  public double DriveKV = 0.18;

  public Translation2d ModuleLocation = new Translation2d();

  public double DriveGearRatio = 6.75;
  public double DriveWheelDiameterMeters = 0.102;
  public double DriveWheelCircumferenceMeters =
    Math.PI * DriveWheelDiameterMeters;

  public Slot0Configs DriveSlot0Configuration;

  public ClosedLoopRampsConfigs DriveClosedLoopRampConfiguration = new ClosedLoopRampsConfigs()
    .withTorqueClosedLoopRampPeriod(0.5)
    .withVoltageClosedLoopRampPeriod(0.5)
    .withDutyCycleClosedLoopRampPeriod(0.5);
  public CurrentLimitsConfigs DriveCurrentLimitConfiguration = new CurrentLimitsConfigs()
    .withStatorCurrentLimitEnable(true)
    .withSupplyCurrentLimit(40)
    .withSupplyCurrentThreshold(50)
    .withSupplyTimeThreshold(100);

  public SwerveModuleConfig(
    String moduleName,
    int driveMotorCanId,
    int steeringMotorCanId,
    int canCoderCanId,
    double startingOffset,
    boolean driveInverted,
    boolean steerInverted,
    PIDConstants drivePid,
    PIDConstants steerPid,
    Translation2d location
  ) {
    ModuleName = moduleName;
    DriveMotorCanId = driveMotorCanId;
    SteeringMotorCanId = steeringMotorCanId;
    CANCoderCanId = canCoderCanId;
    StartingOffset = startingOffset;
    DriveInverted = driveInverted;
    SteerInverted = steerInverted;
    DriveSlot0Configuration =
      new Slot0Configs()
        .withKP(drivePid.kP)
        .withKI(drivePid.kI)
        .withKD(drivePid.kD)
        .withKV(0.105);
    SteeringPidConstants = steerPid;
    ModuleLocation = location;
  }
}
