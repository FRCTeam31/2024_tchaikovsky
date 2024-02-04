package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements AutoCloseable {

  TalonFX m_shooterFalconFX;
  RobotConfig m_robotConfig;

  // Creates a new Shooter
  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    m_shooterFalconFX = new TalonFX(RobotConfig.m_shooterFalconFXCanID);
  }

  // Method for giving the Shooter Motor a speed
  public void runShooter(double speed) {
    m_shooterFalconFX.set(speed);
  }

  // Command for stopping the shooter motors
  public Command stopMotorsCommand() {
    return this.run(() -> {
        m_shooterFalconFX.stopMotor();
      });
  }

  public Command runMotorsCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runShooter(speed.getAsDouble());
      });
  }

  public void close() {
    m_shooterFalconFX.close();
  }
}
