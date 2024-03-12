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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Set LED startup pattern
    m_robotContainer.LEDs.setSection(0, LEDSection.pulseColor(Color.ORANGE, 100));
  }

  @Override
  public void disabledInit() {
    // Set disabled LED pattern
    m_robotContainer.LEDs.setSection(0, LEDSection.pulseColor(onRedAlliance() ? Color.RED : Color.BLUE, 100));
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
    // Set auto LED pattern
    m_robotContainer.LEDs.setSection(0, LEDSection.blinkColor(onRedAlliance() ? Color.RED : Color.BLUE, 100));

    var autoCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = autoCommand; // Save the command for cancelling later if needed

    // Exit without scheduling an auto command if none is selected
    if (autoCommand == null || autoCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      // reset the gyro to 0 or 180 depending on which alliance we're on
      m_robotContainer.Drivetrain.m_gyro.setYaw(onRedAlliance() ? 0 : 180);
      return;
    }

    // Get the auto's starting pose and reset the gyro to the initial direction the robot is facing
    var startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoCommand.getName());
    m_robotContainer.Drivetrain.m_gyro.setYaw(startingPose.getRotation().getDegrees());

    // Schedule the auto command
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
    m_robotContainer.LEDs.setSection(0, LEDSection.raceColor(onRedAlliance() ? Color.RED : Color.BLUE, 25, true));
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
    return DriverStation.getAlliance().get() == Alliance.Red;
  }

  public static boolean onBlueAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }
}
