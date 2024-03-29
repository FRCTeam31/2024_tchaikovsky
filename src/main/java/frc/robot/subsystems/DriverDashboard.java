package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig;
import java.util.Map;

public class DriverDashboard {

  public ShuffleboardTab DriverTab = Shuffleboard.getTab("Driver");
  public ShuffleboardTab AutoTab = Shuffleboard.getTab("Auto Commands");

  public SendableChooser<Command> AutoChooser;
  public GenericEntry AllianceBox = DriverTab
    .add("Alliance", false)
    .withPosition(13, 0)
    .withSize(2, 3)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#0000FF"))
    .getEntry();

  // Drive
  public Field2d FieldWidget = new Field2d();
  public GenericEntry HeadingGyro = DriverTab
    .add("Current Heading", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(8, 3)
    .withSize(3, 3)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();
  public GenericEntry RearApTagIdField = DriverTab
    .add("Rear APTag", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(11, 3)
    .withSize(2, 1)
    .getEntry();
  public GenericEntry FrontApTagIdField = DriverTab
    .add("Front APTag", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(13, 3)
    .withSize(2, 1)
    .getEntry();
  public GenericEntry RearApTagOffsetDial = DriverTab
    .add("Rear APTag X Offset", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("Min", -29.8, "Max", 29.8))
    .withPosition(11, 4)
    .withSize(2, 3)
    .getEntry();
  public GenericEntry FrontPoseEstimationSwitch = DriverTab
    .add("F Pose Est.", false)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .withPosition(13, 4)
    .withSize(2, 1)
    .getEntry();
  public GenericEntry RearPoseEstimationSwitch = DriverTab
    .add("R Pose Est.", true)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .withPosition(13, 5)
    .withSize(2, 1)
    .getEntry();

  // Climbers
  public GenericEntry ClimberControlsActiveBox = DriverTab
    .add("Climbers Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(5, 4)
    .withSize(3, 2)
    .getEntry();

  /**
   * Constructs a new DriverDashboard and adds complex widgets that must be created in the constructor
   * @param config
   */
  public DriverDashboard(RobotConfig config) {
    DriverTab
      .addCamera("Rear Stream", "LLRear", "http://" + config.Drivetrain.LimelightRearName + ".local:5800/stream.mjpg")
      .withPosition(0, 0)
      .withSize(8, 4)
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("Show controls", false, "Show crosshair", false));
    // DriverTab
    //   .addCamera(
    //     "Front Stream",
    //     config.Drivetrain.LimelightFrontName,
    //     "http://" + config.Drivetrain.LimelightFrontName + ".local:5800/stream.mjpg"
    //   )
    //   .withSize(4, 4)
    //   .withPosition(5, 0)
    //   .withWidget(BuiltInWidgets.kCameraStream)
    //   .withProperties(Map.of("Show controls", false, "Show crosshair", false));

    DriverTab.add("Field", FieldWidget).withWidget(BuiltInWidgets.kField).withPosition(8, 0).withSize(5, 3);
  }

  /**
   * Adds an auto chooser to the Shuffleboard and configures it
   * @param chooser
   */
  public void addAutoChooser(SendableChooser<Command> chooser) {
    AutoChooser = chooser;
    DriverTab.add(AutoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 4).withSize(5, 2);
  }
}
