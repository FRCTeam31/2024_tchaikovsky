// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;
import prime.movers.LazyCANSparkMax;

public class Shooter extends SubsystemBase implements AutoCloseable {

  private LazyCANSparkMax m_shooterSparkMax;
  RobotConfig m_robotConfig;

  // Creates a new Shooter
  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    m_shooterSparkMax =
      new LazyCANSparkMax(
        m_robotConfig.m_shooterSparkMaxCanID,
        MotorType.kBrushless
      );
  }

  // Method for giving the Shooter Motor a speed
  public void runShooter(double speed) {
    m_shooterSparkMax.set(speed);
  }

  // Command for stopping the shooter motors
  public Command stopMotorsCommand() {
    return this.run(() -> {
        m_shooterSparkMax.stopMotor();
      });
  }

  public Command runMotorsCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runShooter(speed.getAsDouble());
      });
  }

  public void close() {
    m_shooterSparkMax.close();
  }
}
