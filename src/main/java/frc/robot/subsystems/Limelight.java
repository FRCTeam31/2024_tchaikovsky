// wip

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    .withProperties(Map.of("30", 50, "Min", -30))
    .getEntry();
  private GenericEntry d_tyEntry = d_tab
    .add("Vertical Target Offset", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("25", 50, "Min", -25))
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
  private GenericEntry d_clEntry = d_tab
    .add("Capture Latency (ms)", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  public Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  //#region Basic Targeting Data

  /**
   * Returns Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
   * @return
   */
  public Rotation2d getHorizontalOffsetFromTarget() {
    return Rotation2d.fromDegrees(
      m_limelightTable.getEntry("tx").getDouble(0.0)
    );
  }

  /**
   * Returns Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
   * @return
   */
  public Rotation2d getVerticalOffsetFromTarget() {
    return Rotation2d.fromDegrees(
      m_limelightTable.getEntry("ty").getDouble(0.0)
    );
  }

  /**
   * Returns Target Area (0% of image to 100% of image)
   * @return
   */
  public double getTargetArea() {
    return m_limelightTable.getEntry("ta").getDouble(0.0);
  }

  /**
   * The pipeline's latency contribution (ms). Add to "cl" to get total latency.
   * @return
   */
  public long getPipelineLatencyMs() {
    return (long) m_limelightTable.getEntry("tl").getDouble(0.0);
  }

  /**
   * Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
   * @return
   */
  public long getCapturePipelineLatencyMs() {
    return (long) m_limelightTable.getEntry("cl").getDouble(0.0);
  }

  //#endregion

  //#region AprilTag and 3D Data

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

  //#endregion

  public void periodic() {
    d_txEntry.setDouble(getHorizontalOffsetFromTarget().getDegrees());
    d_tyEntry.setDouble(getVerticalOffsetFromTarget().getDegrees());
    d_taEntry.setDouble(getTargetArea());
    d_tlEntry.setDouble(getPipelineLatencyMs());
    d_clEntry.setDouble(getCapturePipelineLatencyMs());
  }
}
