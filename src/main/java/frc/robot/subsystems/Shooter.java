package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements AutoCloseable {

  TalonFX m_shooter;
  RobotConfig m_config;

  // Creates a new Shooter
  public Shooter(RobotConfig robotConfig) {
    m_config = robotConfig;

    m_shooter = new TalonFX(RobotConfig.m_shooterFalconFXCanID);
    m_shooter.getConfigurator().apply(new TalonFXConfiguration()); // Reset to factory default
  }

  // Method for giving the Shooter Motor a speed
  public void runShooter(double speed) {
    m_shooter.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "Shooter velocity",
      m_shooter.getVelocity().getValueAsDouble()
    );
  }

  // Command for stopping the shooter motors
  public Command stopMotorsCommand() {
    return this.run(() -> {
        m_shooter.stopMotor();
      });
  }

  public Command runMotorsCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runShooter(speed.getAsDouble());
      });
  }

  public void close() {
    m_shooter.close();
  }
}
