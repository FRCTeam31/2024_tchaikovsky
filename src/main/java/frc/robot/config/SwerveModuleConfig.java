package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleConfig {

  public String ModuleName = "DefaultModuleName";

  public int DriveMotorCanId = 0;
  public int SteeringMotorCanId = 0;
  public int CANCoderCanId = 0;

  public double StartingOffset = 0;
  public boolean DriveInverted = false;
  public boolean SteerInverted = false;

  public double ModuleLocationXMeters = 0;
  public double ModuleLocationYMeters = 0;
  public Translation2d ModuleLocation = new Translation2d();

  public double DriveGearRatio = 6.75;
  public double DriveWheelDiameterMeters = 0.102;
  public double DriveWheelCircumferenceMeters =
    Math.PI * DriveWheelDiameterMeters;

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
    Translation2d location
  ) {
    ModuleName = moduleName;
    DriveMotorCanId = driveMotorCanId;
    SteeringMotorCanId = steeringMotorCanId;
    CANCoderCanId = canCoderCanId;
    StartingOffset = startingOffset;
    DriveInverted = driveInverted;
    SteerInverted = steerInverted;
    ModuleLocationXMeters = location.getX();
    ModuleLocationYMeters = location.getY();
    ModuleLocation = location;
  }

  public Translation2d getModuleLocation() {
    return new Translation2d(ModuleLocationXMeters, ModuleLocationYMeters);
  }
}
