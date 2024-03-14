// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import java.util.Map;
import prime.control.Controls;
import prime.control.HolonomicControlStyle;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDSection;
import prime.control.PrimeXboxController;

public class RobotContainer {

  private RobotConfig m_config;
  private PrimeXboxController m_driverController;
  private PrimeXboxController m_operatorController;

  public ShuffleboardTab d_driverTab = Shuffleboard.getTab("Driver");
  // public ShuffleboardTab d_autoTab = Shuffleboard.getTab("Auto");

  private SendableChooser<Command> m_autoChooser;
  public GenericEntry d_allianceEntry = d_driverTab
    .add("Alliance Color", false)
    .withSize(3, 0)
    .withPosition(0, 4)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#0000FF"))
    .getEntry();

  public Drivetrain Drivetrain;
  public Shooter Shooter;
  public Intake Intake;
  public Climbers Climbers;
  public Limelight Limelight;
  public LEDs LEDs;
  public Compressor Compressor;

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
      if (Compressor != null) Compressor.close();

      // Create new subsystems
      LEDs = new LEDs(m_config.LEDs);
      Drivetrain = new Drivetrain(m_config);
      Shooter = new Shooter(m_config.Shooter, LEDs);
      Intake = new Intake(m_config.Intake);
      Climbers = new Climbers(m_config.Climbers);
      Limelight = new Limelight(m_config.LimelightPose);
      Compressor = new Compressor(m_config.PneumaticsModuleId, PneumaticsModuleType.REVPH);
      Compressor.enableDigital();

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
      .withPosition(0, 0)
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("Show controls", false, "Show crosshair", false));

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_autoChooser = AutoBuilder.buildAutoChooser("Park Auto");
    d_driverTab.add(m_autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(3, 1).withPosition(1, 4);
    // var possibleAutos = AutoBuilder.getAllAutoNames();
    // for (int i = 0; i < possibleAutos.size(); i++) {
    //   var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
    //   d_autoTab.add(possibleAutos.get(i), autoCommand).withWidget(BuiltInWidgets.kCommand).withSize(2, 1);
    // }

    // TODO: Add more important items from subsystems here
    d_driverTab
      .add("Field", Drivetrain.m_fieldWidget)
      .withWidget(BuiltInWidgets.kField)
      .withPosition(8, 0)
      .withSize(5, 3);
    d_driverTab.add("Robot Gyro", Drivetrain.m_gyro).withWidget(BuiltInWidgets.kGyro).withPosition(8, 3).withSize(2, 2);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  /**
   * Creates the controller and configures the driver's controls
   */
  public void configureDriverControls() {
    // Controls for Driving
    m_driverController.a().onTrue(Drivetrain.resetGyroCommand());
    Drivetrain.setDefaultCommand(
      Drivetrain.defaultDriveCommand(
        m_driverController.getSwerveControlProfile(
          HolonomicControlStyle.Drone,
          m_config.Drivetrain.DriveDeadband,
          m_config.Drivetrain.DeadbandCurveWeight
        ),
        true
      )
    );

    // While holding b, auto-aim the robot to the apriltag target using snap-to
    m_driverController
      .leftBumper()
      .whileTrue(
        Commands.run(
          () -> {
            var targetedAprilTag = Limelight.getApriltagId();

            // If targetedAprilTag is in validTargets, snap to its offset
            if (targetedAprilTag != -1 && Limelight.tagIdIsASpeakerTarget(targetedAprilTag)) {
              // Calculate the target heading
              var horizontalOffset = Limelight.getHorizontalOffsetFromTarget().getDegrees();
              var robotHeading = Drivetrain.getHeading();
              var targetHeading = robotHeading + horizontalOffset;

              // Set the drivetrain to snap to the target heading
              Drivetrain.setSnapToSetpoint(targetHeading);

              // If the target is within 5 degrees, set the LEDs to indicate shoot, otherwise quickly pulse red
              if (Math.abs(horizontalOffset) < 5) {
                LEDs.setStripTemporary(LEDSection.solidColor(Color.GREEN));
              } else {
                LEDs.setStripTemporary(LEDSection.pulseColor(Color.RED, 100));
              }
            } else {
              Drivetrain.setSnapToGyroEnabled(false);
              LEDs.restoreLastStripState();
            }
          },
          Drivetrain
        )
      )
      .onFalse(Drivetrain.disableSnapTo().andThen(Commands.runOnce(() -> LEDs.restoreLastStripState())));

    // Controls for Snap-To with field-relative setpoints
    m_driverController.x().onTrue(Drivetrain.disableSnapTo());
    m_driverController.pov(Controls.up).onTrue(Drivetrain.setSnapToSetpointCommand(Math.toRadians(0)));
    m_driverController.pov(Controls.left).onTrue(Drivetrain.setSnapToSetpointCommand(Math.toRadians(90)));
    m_driverController.pov(Controls.down).onTrue(Drivetrain.setSnapToSetpointCommand(Math.toRadians(180)));
    m_driverController.pov(Controls.right).onTrue(Drivetrain.setSnapToSetpointCommand(Math.toRadians(270)));

    // Climbers
    m_driverController.y().onTrue(Climbers.toggleClimbControlsCommand());
    m_driverController.start().onTrue(Climbers.setArmsUpCommand());
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
    // Intake ========================================
    m_operatorController.a().onTrue(Intake.toggleIntakeInAndOutCommand()); // Set intake angle in/out

    m_operatorController // When the trigger is pressed, intake a note at a variable speed
      .leftTrigger(0.1)
      .whileTrue(Intake.runRollersAtSpeedCommand(() -> m_operatorController.getLeftTriggerAxis()))
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
