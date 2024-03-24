// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import java.util.Map;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.BlinkPattern;
import prime.control.LEDs.Patterns.ChasePattern;
import prime.control.LEDs.Patterns.PulsePattern;

public class Robot extends TimedRobot {

  public enum Side {
    kLeft,
    kRight,
  }

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private GenericEntry d_allianceEntry;

  @Override
  public void robotInit() {
    // Start L2 logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_robotContainer = new RobotContainer(RobotConfig.getDefault());
    d_allianceEntry =
      RobotContainer.DriverTab
        .add("Alliance", false)
        .withSize(2, 3)
        .withPosition(13, 0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#0000FF"))
        .getEntry();
  }

  @Override
  public void disabledInit() {
    // Set disabled LED pattern
    m_robotContainer.LEDs.setStripPersistentPattern(new PulsePattern(onRedAlliance() ? Color.RED : Color.BLUE, 2));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    d_allianceEntry.setBoolean(onRedAlliance());
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      // Cancel the auto command if it's still running
      m_autonomousCommand.cancel();

      // Stop the shooter and intake motors in case they're still running
      m_robotContainer.Shooter.stopMotorsCommand().schedule();
      m_robotContainer.Intake.stopRollersCommand().schedule();
    }

    // Set auto LED pattern
    m_robotContainer.LEDs.setStripPersistentPattern(new BlinkPattern(onRedAlliance() ? Color.RED : Color.BLUE, 0.15));

    var autoCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = autoCommand; // Save the command for cancelling later if needed

    // Exit without scheduling an auto command if none is selected
    if (autoCommand == null || autoCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      m_robotContainer.Drivetrain.resetGyro();
      return;
    }

    // Schedule the auto command
    m_robotContainer.Drivetrain.EnableContinuousPoseEstimation = true;
    if (onRedAlliance()) m_robotContainer.Drivetrain.resetGyro();

    SmartDashboard.putString("Robot/Auto/CommandName", autoCommand.getName());
    autoCommand.schedule();
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      // Cancel the auto command if it's still running
      m_autonomousCommand.cancel();

      // Stop the shooter and intake motors in case they're still running
      m_robotContainer.Shooter.stopMotorsCommand().schedule();
      m_robotContainer.Intake.stopRollersCommand().schedule();
    }

    // Set teleop LED pattern
    m_robotContainer.LEDs.setStripPersistentPattern(
      new ChasePattern(onRedAlliance() ? Color.RED : Color.BLUE, 0.75, false)
    );

    m_robotContainer.Drivetrain.EnableContinuousPoseEstimation = false;
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    // Start the data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public static boolean onRedAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static boolean onBlueAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Blue;
  }
}
