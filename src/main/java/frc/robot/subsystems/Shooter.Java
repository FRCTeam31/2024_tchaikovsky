// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotConfig;
import java.util.Map;
import java.util.function.DoubleSupplier;
import prime.movers.LinearActuator;

public class Shooter extends SubsystemBase implements AutoCloseable {

  TalonFX m_shooterTalonFX;
  RobotConfig m_robotConfig;
  private VictorSPX m_shooterVictorSPX;
  LinearActuator m_leftLinearActuator;
  LinearActuator m_rightLinearActuator;
  private ShuffleboardTab d_shooterTab = Shuffleboard.getTab("Shooter");
  private GenericEntry d_leftLinearActuator = d_shooterTab
    .add("Left Actuator Position", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Min", 0, "Max", 1))
    .getEntry();

  private GenericEntry d_rightLinearActuator = d_shooterTab
    .add("Right Actuator Position", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Min", 0, "Max", 1))
    .getEntry();

  // Creates a new Shooter
  public Shooter(RobotConfig robotConfig) {
    m_robotConfig = robotConfig;
    m_shooterTalonFX = new TalonFX(robotConfig.m_shooterTalonFXCanID);
    m_shooterVictorSPX = new VictorSPX(robotConfig.m_shooterVictorSPXCanID);
    m_leftLinearActuator = new LinearActuator(0, 0);
    m_rightLinearActuator = new LinearActuator(1, 1);
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

  @Override
  public void periodic() {
    d_leftLinearActuator.setDouble(m_leftLinearActuator.getPosition());
    d_rightLinearActuator.setDouble(m_rightLinearActuator.getPosition());
  }

  public void close() {
    m_shooterTalonFX.close();
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

  public Command RaiseActuatorsCommand() {
    return this.run(() -> {
        m_leftLinearActuator.runForward();
        m_rightLinearActuator.runForward();
      });
  }

  public Command LowerActuatorsCommand() {
    return this.run(() -> {
        m_leftLinearActuator.runReverse();
        m_rightLinearActuator.runReverse();
      });
  }

  public Command stopActuatorsCommand() {
    return this.run(() -> {
        m_leftLinearActuator.stop();
        m_rightLinearActuator.stop();
      });
  }
  //#endregion
}
