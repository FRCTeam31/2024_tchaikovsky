// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {

  TalonFX m_shooterFalconFX;
  RobotConfig m_robotConfig;

  /** Creates a new Shooter. */
  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    m_shooterFalconFX = new TalonFX(RobotConfig.m_shooterFalconFXCanID);
  }

  public void runShooter(double speed) {
    m_shooterFalconFX.set(speed);
  }

  public Command RunShooterCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runShooter(speed.getAsDouble());
      });
  }
}
