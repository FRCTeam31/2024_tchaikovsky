// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDSection;

public class Robot extends TimedRobot {

  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(RobotConfig.getDefault());

    // Set startup pattern
    var modePattern = LEDSection.pulseColor(Color.ORANGE, 1000);
    m_robotContainer.LEDs.setLeftSection(0, modePattern);
    m_robotContainer.LEDs.setRightSection(0, modePattern);
  }

  @Override
  public void disabledInit() {
    var modePattern = LEDSection.pulseColor(onRedAlliance() ? Color.RED : Color.BLUE, 255);
    m_robotContainer.LEDs.setLeftSection(0, modePattern);
    m_robotContainer.LEDs.setRightSection(0, modePattern);
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
    var modePattern = LEDSection.blinkColor(onRedAlliance() ? Color.RED : Color.BLUE, 250);
    m_robotContainer.LEDs.setLeftSection(0, modePattern);
    m_robotContainer.LEDs.setRightSection(0, modePattern);

    // Get the selected auto command
    var autoCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = autoCommand; // Save the command for cancelling later if needed

    // Exit without scheduling an auto command if none is selected
    if (autoCommand == null || autoCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      return;
    }

    var startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoCommand.getName());
    m_robotContainer.Drivetrain.resetGyro();
    m_robotContainer.Drivetrain.resetOdometry(startingPose);
    autoCommand.schedule();
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    var modePattern = LEDSection.raceColor(onRedAlliance() ? Color.RED : Color.BLUE, 500, true);
    m_robotContainer.LEDs.setLeftSection(0, modePattern);
    m_robotContainer.LEDs.setRightSection(0, modePattern);
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.configureTestControls();

    // Start the data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public static boolean onRedAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static boolean onBlueAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }
}
