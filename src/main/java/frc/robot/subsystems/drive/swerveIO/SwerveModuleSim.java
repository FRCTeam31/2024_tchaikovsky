package frc.robot.subsystems.drive.swerveIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.config.DrivetrainConfig;
import frc.robot.config.SwerveModuleConfig;
import frc.robot.sim.MotorSim;

public class SwerveModuleSim implements ISwerveModule {

  private final MotorSim _driveMotor;
  private final MotorSim _steerMotor;
  private double _steerAngle = 0;
  private double _maxSpeedMetersPerSecond;

  public SwerveModuleSim(DrivetrainConfig driveConfig, SwerveModuleConfig moduleConfig) {
    _maxSpeedMetersPerSecond = driveConfig.MaxSpeedMetersPerSecond;
    _driveMotor = new MotorSim(driveConfig.MaxSpeedMetersPerSecond);
    _steerMotor = new MotorSim(2 * Math.PI);
  }

  @Override
  public void setDesiredState(SwerveModuleState state) {
    _driveMotor.set(state.speedMetersPerSecond / _maxSpeedMetersPerSecond);
    _steerAngle = state.angle.getRadians();
  }

  @Override
  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(_driveMotor.get() * _maxSpeedMetersPerSecond, Rotation2d.fromRadians(_steerAngle));
  }

  @Override
  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(_driveMotor.getDistance(), Rotation2d.fromRadians(_steerAngle));
  }

  @Override
  public void stopMotors() {
    _driveMotor.set(0);
    _steerMotor.set(0);
  }
}
