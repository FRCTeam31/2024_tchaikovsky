// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrips;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import java.util.Map;
import prime.control.Controls;
import prime.control.PrimeXboxController;

public class RobotContainer {

  private RobotConfig m_config;
  private PrimeXboxController m_driverController;
  private PrimeXboxController m_operatorController;

  public ShuffleboardTab d_driverTab = Shuffleboard.getTab("Driver");
  public ShuffleboardTab d_autoTab = Shuffleboard.getTab("Auto");

  private SendableChooser<Command> m_autoChooser;
  public GenericEntry d_allianceEntry = d_driverTab
    .add("Alliance Color", false)
    .withSize(3, 0)
    .withPosition(12, 0)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#0000FF"))
    .getEntry();

  public Drivetrain Drivetrain;
  public Shooter Shooter;
  public Intake Intake;
  public Climbers Climbers;
  public Limelight Limelight;
  public LEDStrips LEDs;

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

      // Save new config
      m_config = config;

      // Close subsystems before reconfiguring
      if (Drivetrain != null) Drivetrain.close();
      if (Shooter != null) Shooter.close();
      if (Intake != null) Intake.close();
      if (Climbers != null) Climbers.close();
      if (LEDs != null) LEDs.close();

      // Create new subsystems
      Drivetrain = new Drivetrain(m_config);
      Shooter = new Shooter(m_config.Shooter);
      Intake = new Intake(m_config.Intake);
      Climbers = new Climbers(m_config.Climbers);
      Limelight = new Limelight(m_config.LimelightPose);
      LEDs = new LEDStrips(m_config.LEDs);

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Intake.getNamedCommands());
      NamedCommands.registerCommands(Shooter.getNamedCommands());
      NamedCommands.registerCommands(CombinedCommands.getNamedCommands(Shooter, Intake)); // Register the combined named commands that use multiple subsystems

      // Create driver dashboard
      configureRobotDashboard();

      // Reconfigure bindings
      configureDriverControls();
      configureOperatorControls();
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to configure robot: " + e.getMessage(), e.getStackTrace());
    }
  }

  public void configureRobotDashboard() {
    d_driverTab
      .addCamera("Limelight Stream", "LL2", "http://limelight.local:5800/stream.mjpg")
      .withSize(8, 4)
      .withPosition(3, 0)
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("Show controls", false, "Show crosshair", false));

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_autoChooser = AutoBuilder.buildAutoChooser("Park Auto");
    var possibleAutos = AutoBuilder.getAllAutoNames();
    for (int i = 0; i < possibleAutos.size(); i++) {
      var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
      d_autoTab.add(possibleAutos.get(i), autoCommand).withWidget(BuiltInWidgets.kCommand).withSize(2, 1);
    }

    d_driverTab.add(m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(3, 1).withPosition(3, 4);
    // TODO: Add more important items from subsystems here
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  /**
   * Creates the controller and configures the driver's controls
   */
  public void configureDriverControls() {
    // Controls for Driving
    Drivetrain.setDefaultCommand(
      Drivetrain.defaultDriveCommand(
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

    m_driverController.a().onTrue(Drivetrain.resetGyroCommand());

    // Controls for Snap-To
    m_driverController.x().onTrue(Commands.runOnce(() -> Drivetrain.setSnapToGyroControl(false)));
    m_driverController.pov(Controls.up).onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(0)));
    m_driverController.pov(Controls.left).onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(270)));
    m_driverController.pov(Controls.down).onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(180)));
    m_driverController.pov(Controls.right).onTrue(Drivetrain.setSnapToSetpoint(Math.toRadians(90)));

    // Climbers
    m_driverController.y().onTrue(Climbers.toggleClimbControlsCommand());
    Climbers.setDefaultCommand(
      Climbers.defaultClimbingCommand(
        m_driverController.button(Controls.RB),
        m_driverController.button(Controls.LB),
        () -> m_driverController.getRawAxis(Controls.RIGHT_TRIGGER),
        () -> m_driverController.getRawAxis(Controls.LEFT_TRIGGER)
      )
    );
  }

  /**
   * Creates the controller and configures the operator's controls
   */
  public void configureOperatorControls() {
    // Default commands for seeking PID setpoints
    Intake.setDefaultCommand(Intake.seekAngleSetpointCommand());
    Shooter.setDefaultCommand(Shooter.seekElevationSetpointCommand());

    // Intake ========================================
    m_operatorController.a().onTrue(Intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out

    m_operatorController // When the trigger is pressed, intake a note at a variable speed
      .leftTrigger(0.1)
      .whileTrue(Intake.runRollersWithSpeedCommand(() -> m_operatorController.getLeftTriggerAxis()))
      .onFalse(Intake.stopRollersCommand());

    m_operatorController // When the trigger is pressed, eject a note at a constant speed
      .rightTrigger(0.1)
      .whileTrue(Intake.ejectNoteCommand())
      .onFalse(Intake.stopRollersCommand());

    // Shooter ========================================
    m_operatorController // Toggle the elevation of the shooter
      .rightBumper()
      .onTrue(Shooter.toggleElevationCommand());

    m_operatorController // Runs only the shooter motors at a constant speed to score in the amp
      .x()
      .whileTrue(Shooter.startShootingNoteCommand())
      .onFalse(Shooter.stopMotorsCommand());

    // Combined shooter and intake commands ===========
    m_operatorController // score in speaker
      .b()
      .onTrue(
        Shooter
          // .scoreInSpeakerCommand()
          .startShootingNoteCommand()
          .andThen(new WaitCommand(0.75))
          .andThen(Intake.ejectNoteCommand())
          .andThen(new WaitCommand(0.75))
          .andThen(Shooter.stopMotorsCommand())
          .andThen(Intake.stopRollersCommand())
      );

    m_operatorController // Run sequence to load a note into the shooter for scoring in the amp
      .y()
      .onTrue(CombinedCommands.loadNoteForAmp(Shooter, Intake));
  }

  /**
   * Configures the controllers and binds test & SysID commands to buttons
   */
  public void configureTestControls() {
    m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);

    m_driverController
      .start()
      .whileTrue(Drivetrain.sysIdQuasistatic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));

    m_driverController
      .back()
      .whileTrue(Drivetrain.sysIdQuasistatic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));

    m_driverController
      .rightBumper()
      .whileTrue(Drivetrain.sysIdDynamic(Direction.kForward))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));

    m_driverController
      .leftBumper()
      .whileTrue(Drivetrain.sysIdDynamic(Direction.kReverse))
      .onFalse(Commands.runOnce(() -> Drivetrain.stopMotors(), Drivetrain));
  }

  public class CombinedCommands {

    public static SequentialCommandGroup scoreInSpeakerSequentialGroup(Shooter shooter, Intake intake) {
      // return shooter
      //   .startShootingNoteCommand() // Start the shooter
      //   .andThen(new WaitCommand(0.75)) // Give it time to reach speed TODO: Velocity control later?
      //   .andThen(intake.ejectNoteCommand()) // Eject from the intake into the shooter
      //   .andThen(new WaitCommand(0.2)) // Give the note time to get into the shooter
      //   .andThen(new WaitUntilCommand(() -> !shooter.isNoteLoaded()))
      //   .withTimeout(0.5) // Wait until the note is shot with a max time
      //   .andThen(new WaitCommand(0.1)) // Give the note time to get fully out of the shooter
      //   .andThen(stopShooterAndIntakeCommand(shooter, intake)); // Stop both the shooter and intake
      return shooter
        .startShootingNoteCommand()
        .andThen(new WaitCommand(0.75))
        .andThen(intake.ejectNoteCommand())
        .andThen(new WaitCommand(0.75))
        .andThen(shooter.stopMotorsCommand())
        .andThen(intake.stopRollersCommand());
    }

    public static SequentialCommandGroup loadNoteForAmp(Shooter shooter, Intake intake) {
      return Commands
        .runOnce(() -> intake.runIntakeRollers(-0.6)) // Eject from the intake
        .alongWith(Commands.runOnce(() -> shooter.runShooter(0.10))) // Load into the shooter
        .andThen(new WaitUntilCommand(shooter::isNoteLoaded))
        .withTimeout(1) // Wait until the note is loaded
        .andThen(new WaitCommand(0.075)) // Give the note time to get into the shooter
        .andThen(stopShooterAndIntakeCommand(shooter, intake)); // Stop both the shooter and intake
    }

    public static SequentialCommandGroup stopShooterAndIntakeCommand(Shooter shooter, Intake intake) {
      return shooter.stopMotorsCommand().andThen(intake.stopRollersCommand());
    }

    public static Map<String, Command> getNamedCommands(Shooter shooter, Intake intake) {
      return Map.of(
        "Score_In_Speaker",
        CombinedCommands.scoreInSpeakerSequentialGroup(shooter, intake),
        "Load_Note_For_Amp",
        CombinedCommands.loadNoteForAmp(shooter, intake),
        "Stop_Shooter_And_Intake",
        CombinedCommands.stopShooterAndIntakeCommand(shooter, intake)
      );
    }
  }
}
