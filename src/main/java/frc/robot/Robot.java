// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.BlinkPattern;
import prime.control.LEDs.Patterns.ChasePattern;
import prime.control.LEDs.Patterns.PulsePattern;
import prime.utilities.PrimeLogFileUtil;

public class Robot extends LoggedRobot {

  public enum Side {
    kLeft,
    kRight,
  }

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    // Configure L3 logging
    Logger.recordMetadata("ProjectName", "31 Tchaikovsky"); // Set a metadata value
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      String logPath = "";

      try {
        logPath = PrimeLogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      } catch (Exception e) {}

      if (logPath.isEmpty() || logPath.contains(".AdvantageScope")) {
        // No path file, so just run simulator
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      } else {
        // Replay using path file
        setUseTiming(false); // Run as fast as possible
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(PrimeLogFileUtil.addPathSuffix(logPath, "_sim"), 0.0000001)); // Save outputs to a new log
      }
    }
    Logger.start();

    var config = RobotConfig.getDefault();
    m_robotContainer = new RobotContainer(config, Robot::isReal);
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

    m_robotContainer.d_allianceEntry.setBoolean(onRedAlliance());
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
