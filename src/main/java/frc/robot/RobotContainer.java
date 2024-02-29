// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import prime.control.Controls;
import prime.control.PrimeXboxController;

public class RobotContainer {

  private RobotConfig m_config;
  private ShuffleboardTab d_robotTab = Shuffleboard.getTab("Robot");

  public PrimeXboxController m_driverController;
  public PrimeXboxController m_operatorController;
  public Drivetrain m_drivetrain;
  public Shooter m_shooter;
  public Intake m_intake;
  public Climbers m_climbers;
  public Limelight m_limelight;
  public LEDStrips m_leds;
  public PowerDistribution m_pdh;

  public RobotContainer(RobotConfig config) {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);
    m_operatorController = new PrimeXboxController(Controls.OPERATOR_PORT);

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
      if (m_shooter != null) m_shooter.close();
      if (m_intake != null) m_intake.close();
      if (m_climbers != null) m_climbers.close();
      if (m_leds != null) m_leds.close();
      if (m_pdh != null) m_pdh.close();

      // Save new config
      m_config = config;

      // Create new subsystems
      m_drivetrain = new Drivetrain(m_config);
      m_shooter = new Shooter(m_config.Shooter);
      m_intake = new Intake(m_config.Intake);
      m_climbers = new Climbers(m_config.Climbers);
      m_limelight = new Limelight(m_config.LimelightPose);
      m_leds = new LEDStrips(m_config.LEDs);
      m_pdh = new PowerDistribution();

      // Reconfigure bindings
      configureTeleopControls();
      // Register the named commands from each subsystem that may be used in PathPlanner
      // NamedCommands.registerCommands(m_drivetrain.getNamedCommands());
      // NamedCommands.registerCommands(m_shooter.getNamedCommands());
      // NamedCommands.registerCommands(m_intake.getNamedCommands());
    } catch (Exception e) {
      DriverStation.reportError(
        "[ERROR] >> Failed to configure robot: " + e.getMessage(),
        e.getStackTrace()
      );
    }
  }

  public void configureRobotDashboard() {
    d_robotTab
      .add("Power Hub", m_pdh)
      .withSize(3, 3)
      .withPosition(3, 0)
      .withWidget(BuiltInWidgets.kPowerDistribution);
  }

  /**
   * Creates the controller and configures teleop controls
   */
  public void configureTeleopControls() {
    // Controls for Driving
    m_drivetrain.setDefaultCommand(
      m_drivetrain.defaultDriveCommand(
        m_driverController.getRightStickYSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        m_driverController.getRightStickXSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        m_driverController.getLeftStickXSupplier(
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        true
      )
    );

    // Controls for Snap-To
    m_driverController
      .pov(Controls.up)
      .onTrue(m_drivetrain.setSnapToSetpoint(Math.toRadians(0)));
    m_driverController
      .pov(Controls.left)
      .onTrue(m_drivetrain.setSnapToSetpoint(Math.toRadians(90)));
    m_driverController
      .pov(Controls.down)
      .onTrue(m_drivetrain.setSnapToSetpoint(Math.toRadians(180)));
    m_driverController
      .pov(Controls.right)
      .onTrue(m_drivetrain.setSnapToSetpoint(Math.toRadians(270)));

    m_driverController
      .button(Controls.A)
      .onTrue(m_drivetrain.resetGyroCommand());
    m_driverController
      .button(Controls.B)
      .onTrue(m_drivetrain.toggleShifterCommand());

    // Climbers
    m_driverController.y().onTrue(m_climbers.toggleClimbControlsCommand());
    m_climbers.setDefaultCommand(
      m_climbers.defaultClimbingCommand(
        m_driverController.button(Controls.RB),
        m_driverController.button(Controls.LB),
        () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER),
        () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER)
      )
    );
    // Operator Controls =================================

    // Always be updating the intake angle PID
    m_intake.setDefaultCommand(m_intake.seekAngleSetpointCommand());
    m_shooter.setDefaultCommand(m_shooter.seekElevationSetpointCommand());

    m_operatorController.a().onTrue(m_intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out // TODO: Test

    m_operatorController // Raise/lower shooter
      .rightBumper()
      .onTrue(m_shooter.toggleElevationCommand()); // wait is integrated

    m_operatorController // Shooting the note
      .b()
      .whileTrue(
        m_shooter
          .scoreInSpeaker()
          .alongWith(
            Commands.run(
              () -> {
                if (!m_shooter.m_shooterIsUp) {
                  m_intake.runIntakeRollers(-1);
                }
              },
              m_intake
            )
          )
      )
      .onFalse(
        m_shooter.stopMotorsCommand().alongWith(m_intake.stopRollersCommand())
      );

    m_operatorController // intake note
      .leftTrigger(0.1)
      .whileTrue(
        m_intake.setRollersSpeedCommand(() ->
          m_operatorController.getLeftTriggerAxis()
        )
      )
      .onFalse(m_intake.stopRollersCommand());

    m_operatorController // eject note
      .rightTrigger(0.1)
      .whileTrue(
        m_intake
          .ejectNoteCommand()
          .alongWith(
            Commands.run(
              () -> {
                if (!m_shooter.m_shooterIsUp && m_intake.m_angleToggledIn) {
                  m_shooter.runShooter(0.3);
                }
              },
              m_shooter
            )
          )
      )
      .onFalse(
        m_intake.stopRollersCommand().alongWith(m_shooter.stopMotorsCommand())
      );
  }

  /**
   * Configures the controllers and binds test & SysID commands to buttons
   */
  public void configureTestControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);

    m_driverController
      .start()
      .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));

    m_driverController
      .back()
      .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));

    m_driverController
      .rightBumper()
      .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));

    m_driverController
      .leftBumper()
      .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> m_drivetrain.stopMotors(), m_drivetrain));
  }
}
