// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.JointSubsystem;
import frc.robot.subsystems.Shooter;
import prime.control.Controls;
import prime.control.PrimeXboxController;

public class RobotContainer {

  public RobotConfig m_config;
  public Drivetrain m_drivetrain;
  public PrimeXboxController m_driverController;
  public PrimeXboxController m_operatorController;
  public Shooter m_shooter;
  public Intake m_intake;
  public JointSubsystem m_jointSubsystem;

  public RobotContainer(RobotConfig config) {
    reconfigure(config);
  }

  /**
   * Stops all commands and reconfigures the robot with a new config instance
   * @param config
   */
  public void reconfigure(RobotConfig config) {
    try {
      // Stop all commands
      CommandScheduler.getInstance().cancelAll();

      // Close subsystems before reconfiguring
      if (m_drivetrain != null) m_drivetrain.close();

      // Save new config
      m_config = config;

      // Create new subsystems
      m_drivetrain = new Drivetrain(m_config);

      // Reconfigure bindings
      configureTeleopControls();
    } catch (Exception e) {
      DriverStation.reportError(
        "[ERROR] >> Failed to reconfigure robot: " + e.getMessage(),
        e.getStackTrace()
      );
    }

    m_shooter = new Shooter(config);
  }

  /**
   * Creates the controller and configures teleop controls
   */
  public void configureTeleopControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
    m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

    m_drivetrain.setDefaultCommand(
      m_drivetrain.defaultDriveCommand(
        m_driverController.getLeftStickYSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        m_driverController.getLeftStickXSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        m_driverController.getTriggerSupplier(),
        true
      )
    );

    m_jointSubsystem.setDefaultCommand(
      m_jointSubsystem.RunShooterCommand(
        m_operatorController.getTriggerSupplier(),
        m_shooter,
        m_intake
      )
    );
    m_intake.setDefaultCommand(
      m_intake.RunIntakeCommand(
        m_operatorController.getRightStickXSupplier(0.1)
      )
    );
    m_intake.setDefaultCommand(m_intake.IntakeAngleCommand(null));
    m_driverController
      .pov(Controls.up)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(0)));
    m_driverController
      .pov(Controls.left)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(90)));
    m_driverController
      .pov(Controls.down)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(180)));
    m_driverController
      .pov(Controls.right)
      .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(270)));

    m_driverController
      .button(Controls.A)
      .onTrue(m_drivetrain.resetGyroCommand());
    m_driverController
      .button(Controls.B)
      .onTrue(m_drivetrain.toggleShifterCommand());
  }
}
