// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements AutoCloseable {

  TalonFX m_shooterTalonFX;
  RobotConfig m_robotConfig;
  private VictorSPX m_shooterVictorSPX;

  // Creates a new Shooter
  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    m_shooterTalonFX = new TalonFX(robotConfig.m_shooterTalonFXCanID);
    m_shooterVictorSPX = new VictorSPX(robotConfig.m_shooterVictorSPXCanID);
  }

  //#region Shooter Methods
  // Method for giving the Shooter Motor a speed
  public void runShooter(double speed) {
    m_shooterTalonFX.set(speed);
    m_shooterVictorSPX.set(VictorSPXControlMode.PercentOutput, speed);
  }

  // Stops the motors
  public void stopMotors() {
    m_shooterTalonFX.stopMotor();
    m_shooterVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  //#endregion

  //#region Shooter Commands
  // Command for stopping the shooter motors
  public Command stopMotorsCommand() {
    return this.run(() -> {
        stopMotors();
      });
  }

  // Command for running the Shooter
  public Command runMotorsCommand(DoubleSupplier speed) {
    return this.run(() -> {
        runShooter(speed.getAsDouble());
      });
  }

  public void close() {
    m_shooterTalonFX.close();
  }
}
//#endregion
