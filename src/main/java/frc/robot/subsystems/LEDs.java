package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.LEDConfig;
import prime.control.LEDs.LEDSection;
import prime.control.LEDs.PrimeLEDController;

public class LEDs extends SubsystemBase implements AutoCloseable {

  private LEDConfig m_config;
  private PrimeLEDController m_controller;
  private LEDSection m_lastStates[];

  public LEDs(LEDConfig config) {
    m_config = config;
    m_lastStates = new LEDSection[m_config.SectionCount];

    try {
      m_controller = new PrimeLEDController(m_config.Port, 3);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize left LEDs - " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Set the state of a section of the LED strip and save it as the last state
   * @param section The section to set
   * @param state The state to set the section to
   */
  private void setSectionAndSave(int section, LEDSection state) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_controller != null) {
      // If the last state isTheSameAs the new state, don't set it
      if (m_lastStates[section] != null && m_lastStates[section].isTheSameAs(state)) {
        return;
      }

      // Set the state and save it
      m_controller.setSectionState((byte) section, state);
      m_lastStates[section] = state;
    }
  }

  /**
   * Set the state of a section of the LED strip without saving it as the last state
   * @param section The section to set
   * @param state The state to set the section to temporarily
   */
  private void setSectionTemporary(int section, LEDSection state) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_controller != null) {
      m_controller.setSectionState((byte) section, state);
    }
  }

  /**
   * Set the states of all sections of the LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setStripAndSave(LEDSection state) {
    if (m_controller != null) {
      for (int i = 0; i < m_config.SectionCount; i++) {
        setSectionAndSave(i, state);
      }
    }
  }

  /**
   * Set the states of all sections of the LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setStripTemporary(LEDSection state) {
    if (m_controller != null) {
      for (int i = 0; i < m_config.SectionCount; i++) {
        setSectionTemporary(i, state);
      }
    }
  }

  /**
   * Set a section of the LED strip back to it's previous state
   * @param section The section to set
   * @param state The state to set the section to
   */
  private void restoreLastSectionState(int section) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_lastStates[section] != null) {
      setSectionAndSave(section, m_lastStates[section]);
    }
  }

  /**
   * Set a section of the LED strip back to it's previous state
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void restoreLastStripState() {
    if (m_lastStates == null) {
      return;
    }

    for (int i = 0; i < m_lastStates.length; i++) {
      restoreLastSectionState(i);
    }
  }

  @Override
  public void close() throws Exception {
    if (m_controller != null) {
      m_controller.close();
    }
  }
}
