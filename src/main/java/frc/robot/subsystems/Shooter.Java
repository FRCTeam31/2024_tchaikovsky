package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import frc.robot.config.RobotConfig;

public class Shooter {

  public TalonFX m_shooterTalonFX;
  public RobotConfig m_robotConfig;

  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    shooterTalonFX = new TalonFX(RobotConfig.shoulderMotorCANID);
  }

  public void runShooter() {
    // shooterTalonFX
  }
}
