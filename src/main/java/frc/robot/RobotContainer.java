// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import prime.control.Controls;
import prime.control.PrimeXboxController;

public class RobotContainer {

  public RobotConfig m_config;
  public PrimeXboxController m_driverController;
  public Drivetrain m_drivetrain;
  public Shooter m_shooter;
  public Intake m_intake;
  public Climbers m_climbers;
  public Limelight m_limelight;

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
      // if (m_drivetrain != null) m_drivetrain.close();
      if (m_shooter != null) m_shooter.close();
      if (m_intake != null) m_intake.close();
      if (m_climbers != null) m_climbers.close();

      // Save new config
      m_config = config;

      // Create new subsystems
      // m_drivetrain = new Drivetrain(m_config);
      m_shooter = new Shooter(m_config.Shooter);
      // m_intake = new Intake(m_config.Intake);
      // m_climbers = new Climbers(m_config.Climbers);
      // m_limelight = new Limelight(m_config.LimelightPose);

      // Reconfigure bindings
      configureTeleopControls();
    } catch (Exception e) {
      DriverStation.reportError(
        "[ERROR] >> Failed to reconfigure robot: " + e.getMessage(),
        e.getStackTrace()
      );
    }
  }

  /**
   * Creates the controller and configures teleop controls
   */
  public void configureTeleopControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
    // m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

    // m_drivetrain.setDefaultCommand(
    //   m_drivetrain.defaultDriveCommand(
    //     m_driverController.getLeftStickYSupplier(
    //       m_config.Drivetrain.DriveDeadband,
    //       m_config.Drivetrain.DeadbandCurveWeight
    //     ),
    //     m_driverController.getLeftStickXSupplier(
    //       m_config.Drivetrain.DriveDeadband,
    //       m_config.Drivetrain.DeadbandCurveWeight
    //     ),
    //     m_driverController.getTriggerSupplier(),
    //     true
    //   )
    // );

    // m_driverController
    //   .pov(Controls.up)
    //   .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(0)));
    // m_driverController
    //   .pov(Controls.left)
    //   .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(90)));
    // m_driverController
    //   .pov(Controls.down)
    //   .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(180)));
    // m_driverController
    //   .pov(Controls.right)
    //   .onTrue(m_drivetrain.driveWithSnapToAngleCommand(Math.toRadians(270)));

    // m_driverController
    //   .button(Controls.A)
    //   .onTrue(m_drivetrain.resetGyroCommand());
    // m_driverController
    //   .button(Controls.B)
    //   .onTrue(m_drivetrain.toggleShifterCommand());ta

    // m_driverController
    //   .button(Controls.Y)
    //   .onTrue(m_climbers.toggleClimbersCommand());
    // m_driverController
    //   .button(Controls.LB)
    //   .onTrue(m_climbers.raiseLeftArmCommand());
    // m_driverController
    //   .button(Controls.RB)
    //   .onTrue(m_climbers.raiseRightArmCommand());
    // m_climbers.setDefaultCommand(
    //   m_climbers.LowerClimbersCommand(
    //     () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER),
    //     () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER)
    //   )
    // );

    // Runs the shooter when the Right Trigger is pressed
    m_shooter.setDefaultCommand(
      m_shooter.runMotorsCommand(() -> m_driverController.getRightTriggerAxis())
    );
    m_driverController
      .button(Controls.A)
      .whileTrue(m_shooter.RaiseActuatorsCommand())
      .onFalse(m_shooter.stopActuatorsCommand());
    m_driverController
      .button(Controls.B)
      .whileTrue(m_shooter.LowerActuatorsCommand())
      .onFalse(m_shooter.stopActuatorsCommand());
    // Load/Shoot
    // m_driverController
    //   .leftBumper()
    //   .whileTrue(
    //     m_shooter
    //       .runMotorsCommand(() -> m_driverController.getRightTriggerAxis())
    //       .alongWith(
    //         m_intake.RunIntakeCommand(() ->
    //           -m_driverController.getRightTriggerAxis()
    //         )
    //       )
    //   );

    //   m_intake.setDefaultCommand(m_intake.runIntakeAnglePid());
    //   m_driverController
    //     .y()
    //     .onTrue(m_intake.setIntakeAngleCommand(m_intake.m_upperLimit / 2));
    //   m_driverController.a().onTrue(m_intake.setIntakeAngleCommand(1));
  }
}
