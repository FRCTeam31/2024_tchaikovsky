package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.LEDConfig;
import prime.control.LEDs.LEDSection;
import prime.control.LEDs.PrimeLEDController;

public class LEDStrips extends SubsystemBase implements AutoCloseable {

  private LEDConfig m_config;
  private PrimeLEDController m_controller;

  public LEDStrips(LEDConfig config) {
    m_config = config;

    try {
      m_controller = new PrimeLEDController(m_config.Port, 3);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize left LEDs - " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Set the state of a section of the left LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setSection(int section, LEDSection state) {
    if (m_controller != null) {
      m_controller.setSectionState((byte) section, state);
    }
  }

  /**
   * Set the state of a section of the left LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setAllSections(LEDSection state) {
    if (m_controller != null) {
      m_controller.setSectionState((byte) 0, state);
      m_controller.setSectionState((byte) 1, state);
      m_controller.setSectionState((byte) 2, state);
    }
  }

  @Override
  public void close() throws Exception {
    if (m_controller != null) {
      m_controller.close();
    }
  }
}
