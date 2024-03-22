package frc.robot.subsystems.drive.gyroIO;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroPigeon2 implements IGyro {

  private Pigeon2 m_gyro;

  public GyroPigeon2(int pidgeonId) {
    m_gyro = new Pigeon2(pidgeonId);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs, double angularVelocity) {
    inputs.Rotation2dCCW = m_gyro.getRotation2d();
  }

  @Override
  public void setYaw(double yaw) {
    m_gyro.setYaw(yaw);
  }
}
