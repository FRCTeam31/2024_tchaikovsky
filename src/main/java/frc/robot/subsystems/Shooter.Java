// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements AutoCloseable {

  TalonFX m_shooterTalonFX;
  RobotConfig m_robotConfig;

  // Creates a new Shooter
  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    m_shooterTalonFX = new TalonFX(robotConfig.m_shooterTalonFXCanID);
  }

  // Method for giving the Shooter Motor a speed
  public void runShooter(double speed) {
    m_shooterTalonFX.set(speed);
  }

  // Command for stopping the shooter motors
  public Command stopMotorsCommand() {
    return this.run(() -> {
        m_shooterTalonFX.stopMotor();
      });
  }

  public Command runMotorsCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runShooter(speed.getAsDouble());
      });
  }

  public void close() {
    m_shooterTalonFX.close();
  }
}
