package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.LEDConfig;
import prime.control.LEDs.LEDSection;
import prime.control.LEDs.PrimeLEDController;

public class LEDStrips extends SubsystemBase implements AutoCloseable {

  private LEDConfig m_config;
  private PrimeLEDController m_leftLedController;
  private PrimeLEDController m_rightLedController;

  public LEDStrips(LEDConfig config) {
    m_config = config;

    try {
      m_leftLedController = new PrimeLEDController(m_config.LeftPort, 3);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize left LEDs - " + e.getMessage(), e.getStackTrace());
    }

    try {
      m_rightLedController = new PrimeLEDController(m_config.RightPort, 3);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize right LEDs - " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Set the state of a section of the left LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setLeftSection(int section, LEDSection state) {
    if (m_leftLedController != null) {
      m_leftLedController.setSectionState((byte) section, state);
    }
  }

  /**
   * Set the state of a section of the right LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setRightSection(int section, LEDSection state) {
    if (m_rightLedController != null) {
      m_rightLedController.setSectionState((byte) section, state);
    }
  }

  @Override
  public void close() throws Exception {
    if (m_leftLedController != null) {
      m_leftLedController.close();
    }

    if (m_rightLedController != null) {
      m_rightLedController.close();
    }
  }
}
