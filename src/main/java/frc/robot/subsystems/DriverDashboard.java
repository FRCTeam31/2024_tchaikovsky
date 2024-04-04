package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.PixelFormat;
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

  public UsbCamera m_frontColorCam;
  public SendableChooser<Command> AutoChooser;
  public GenericEntry AllianceBox = DriverTab
    .add("Alliance", false)
    .withPosition(17, 0)
    .withSize(2, 3)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("Color when true", "#FF0000", "Color when false", "#0000FF"))
    .getEntry();

  // Drive
  public Field2d FieldWidget = new Field2d();
  public GenericEntry HeadingGyro = DriverTab
    .add("Current Heading", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .withPosition(12, 3)
    .withSize(3, 3)
    .withProperties(Map.of("Counter clockwise", true, "Major tick spacing", 45.0, "Minor tick spacing", 15.0))
    .getEntry();
  public GenericEntry RearApTagIdField = DriverTab
    .add("Rear APTag", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(15, 3)
    .withSize(2, 1)
    .getEntry();
  public GenericEntry FrontApTagIdField = DriverTab
    .add("Front APTag", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(17, 3)
    .withSize(2, 1)
    .getEntry();
  public GenericEntry RearApTagOffsetDial = DriverTab
    .add("Rear APTag X Offset", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("Min", -29.8, "Max", 29.8))
    .withPosition(15, 4)
    .withSize(2, 3)
    .getEntry();
  public GenericEntry FrontPoseEstimationSwitch = DriverTab
    .add("F Pose Est.", true)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .withPosition(17, 4)
    .withSize(2, 1)
    .getEntry();
  public GenericEntry RearPoseEstimationSwitch = DriverTab
    .add("R Pose Est.", true)
    .withWidget(BuiltInWidgets.kToggleSwitch)
    .withPosition(17, 5)
    .withSize(2, 1)
    .getEntry();

  // Climbers
  public GenericEntry ClimberControlsActiveBox = DriverTab
    .add("Climbers Enabled", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(5, 6)
    .withSize(3, 2)
    .getEntry();

  /**
   * Constructs a new DriverDashboard and adds complex widgets that must be created in the constructor
   * @param config
   */
  public DriverDashboard(RobotConfig config) {
    // DriverTab
    //   .addCamera(
    //     "Rear Limelight",
    //     "Limelight Rear",
    //     "http://" + config.Drivetrain.LimelightRearName + ".local:5800/stream.mjpg"
    //   )
    //   .withPosition(0, 0)
    //   .withSize(6, 6)
    //   .withWidget(BuiltInWidgets.kCameraStream)
    //   .withProperties(Map.of("Show controls", false, "Show crosshair", false));
    m_frontColorCam = CameraServer.startAutomaticCapture();
    m_frontColorCam.setResolution(640, 480);
    m_frontColorCam.setFPS(20);
    m_frontColorCam.setPixelFormat(PixelFormat.kMJPEG);

    // DriverTab
    //   .add(m_frontColorCam)
    //   .withPosition(6, 0)
    //   .withSize(6, 6)
    //   .withWidget(BuiltInWidgets.kCameraStream)
    //   .withProperties(Map.of("Show controls", false, "Show crosshair", false));

    DriverTab.add("Field", FieldWidget).withWidget(BuiltInWidgets.kField).withPosition(12, 0).withSize(5, 3);
  }

  /**
   * Adds an auto chooser to the Shuffleboard and configures it
   * @param chooser
   */
  public void addAutoChooser(SendableChooser<Command> chooser) {
    AutoChooser = chooser;
    DriverTab.add(AutoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 6).withSize(5, 2);
  }
}
