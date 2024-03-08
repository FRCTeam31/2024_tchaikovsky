// wip

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class Limelight extends SubsystemBase {

  private NetworkTable m_limelightTable;

  private ShuffleboardTab d_tab = Shuffleboard.getTab("Limelight");
  private GenericEntry d_txEntry = d_tab
    .add("Horizontal Target Offset", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("Max", 30, "Min", -30))
    .getEntry();
  private GenericEntry d_tyEntry = d_tab
    .add("Vertical Target Offset", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("Max", 25, "Min", -25))
    .getEntry();
  private GenericEntry d_taEntry = d_tab
    .add("Target Area", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("100", 100, "Min", 0))
    .getEntry();
  private GenericEntry d_tlEntry = d_tab
    .add("Pipeline Latency (ms)", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();
  private GenericEntry d_clEntry = d_tab.add("Capture Latency (ms)", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  private GenericEntry d_tidEntry = d_tab
    .add("Primary In-View AprilTag ID", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  /**
   * Creates a new Limelight subsystem and sets the camera's pose in the coordinate system of the robot.
   * @param cameraPose
   */
  public Limelight(Pose3d cameraPose) {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    setCameraPose(cameraPose);
  }

  //#region Basic Targeting Data

  /**
   * Returns Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
   */
  public Rotation2d getHorizontalOffsetFromTarget() {
    return Rotation2d.fromDegrees(m_limelightTable.getEntry("tx").getDouble(0.0));
  }

  /**
   * Returns Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
   */
  public Rotation2d getVerticalOffsetFromTarget() {
    return Rotation2d.fromDegrees(m_limelightTable.getEntry("ty").getDouble(0.0));
  }

  /**
   * Returns Target Area (0% of image to 100% of image)
   */
  public double getTargetArea() {
    return m_limelightTable.getEntry("ta").getDouble(0.0);
  }

  /**
   * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
   */
  public long getPipelineLatencyMs() {
    return (long) m_limelightTable.getEntry("tl").getDouble(0.0);
  }

  /**
   * Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
   */
  public long getCapturePipelineLatencyMs() {
    return (long) m_limelightTable.getEntry("cl").getDouble(0.0);
  }

  //#endregion

  //#region AprilTag and 3D Data

  /**
   * ID of the primary in-view AprilTag
   */
  public int getApriltagId() {
    return (int) m_limelightTable.getEntry("tid").getDouble(0.0);
  }

  /**
   * Robot transform in field-space.
   */
  public Pose3d getRobotPose() {
    var poseData = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

    return toPose3d(poseData);
  }

  /**
   * Robot transform in field-space (alliance driverstation WPILIB origin).
   * @param alliance
   */
  public Pose3d getRobotPose(DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Blue) {
      var poseData = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

      return toPose3d(poseData);
    } else {
      var poseData = m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

      return toPose3d(poseData);
    }
  }

  /**
   * 3D transform of the robot in the coordinate system of the primary in-view AprilTag
   */
  public Pose3d getRobotPoseInTargetSpace() {
    var poseData = m_limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

    return toPose3d(poseData);
  }

  /**
   * 3D transform of the camera in the coordinate system of the primary in-view AprilTag
   */
  public Pose3d getCameraPoseInTargetSpace() {
    var poseData = m_limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

    return toPose3d(poseData);
  }

  /**
   * 3D transform of the camera in the coordinate system of the robot
   */
  public Pose3d getCameraPoseInRobotSpace() {
    var poseData = m_limelightTable.getEntry("camerapose_robotspace").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

    return toPose3d(poseData);
  }

  /**
   * 3D transform of the primary in-view AprilTag in the coordinate system of the Camera
   */
  public Pose3d getTargetPoseInCameraSpace() {
    var poseData = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

    return toPose3d(poseData);
  }

  /**
   * 3D transform of the primary in-view AprilTag in the coordinate system of the Robot
   */
  public Pose3d getTargetPoseInRobotSpace() {
    var poseData = m_limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); // Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)

    return toPose3d(poseData);
  }

  //#endregion

  //#region Camera Controls

  /**
   * Sets limelight’s LED state.
   *    0 = use the LED Mode set in the current pipeline.
   *    1 = force off.
   *    2 = force blink.
   *    3 = force on.
   * @param mode
   */
  public void setLedMode(int mode) {
    m_limelightTable.getEntry("ledMode").setNumber(mode);
  }

  /**
   * Sets limelight’s operation mode.
   *    0 = Vision processor.
   *    1 = Driver Camera (Increases exposure, disables vision processing).
   * @param mode
   */
  public void setCameraMode(int mode) {
    m_limelightTable.getEntry("camMode").setNumber(mode);
  }

  /**
   * Sets limelight’s pipeline.
   * @param pipeline
   */
  public void setPipeline(int pipeline) {
    m_limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Side-by-side streams (Note: USB output stream is not affected by this mode)
   * @param mode
   */
  public void setPiPStreamingMode(int mode) {
    m_limelightTable.getEntry("stream").setNumber(mode);
  }

  /**
   * Allows users to take snapshots during a match
   * @param mode
   */
  public void takeSnapshot() {
    m_limelightTable.getEntry("snapshot").setNumber(0); // Reset
    m_limelightTable.getEntry("snapshot").setNumber(1); // Take snapshot
    m_limelightTable.getEntry("snapshot").setNumber(0); // Reset
  }

  /**
   * Set the camera's pose in the coordinate system of the robot.
   * @param pose
   */
  public void setCameraPose(Pose3d pose) {
    var poseData = new double[] {
      pose.getTranslation().getX(),
      pose.getTranslation().getY(),
      pose.getTranslation().getZ(),
      Units.radiansToDegrees(pose.getRotation().getX()),
      Units.radiansToDegrees(pose.getRotation().getY()),
      Units.radiansToDegrees(pose.getRotation().getZ()),
    };

    m_limelightTable.getEntry("camerapose_robotspace_set").setDoubleArray(poseData);
  }

  //#endregion

  public void periodic() {
    d_txEntry.setDouble(getHorizontalOffsetFromTarget().getDegrees());
    d_tyEntry.setDouble(getVerticalOffsetFromTarget().getDegrees());
    d_taEntry.setDouble(getTargetArea());
    d_tlEntry.setDouble(getPipelineLatencyMs());
    d_clEntry.setDouble(getCapturePipelineLatencyMs());
    d_tidEntry.setDouble(getApriltagId());
  }

  private Pose3d toPose3d(double[] poseData) {
    if (poseData.length < 6) {
      System.err.println("Bad LL 3D Pose Data!");

      return new Pose3d();
    }

    return new Pose3d(
      new Translation3d(poseData[0], poseData[1], poseData[2]),
      new Rotation3d(
        Units.degreesToRadians(poseData[3]),
        Units.degreesToRadians(poseData[4]),
        Units.degreesToRadians(poseData[5])
      )
    );
  }
}
