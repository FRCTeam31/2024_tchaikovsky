package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.LEDConfig;
import prime.control.LEDs.PrimeLEDController;
import prime.control.LEDs.SectionState;

public class LEDStrips extends SubsystemBase implements AutoCloseable {

  private LEDConfig m_config;
  private PrimeLEDController m_leftLedController;
  private PrimeLEDController m_rightLedController;

  public LEDStrips(LEDConfig config) {
    m_config = config;
    m_leftLedController = new PrimeLEDController(m_config.LeftPort, 3);
    m_rightLedController = new PrimeLEDController(m_config.RightPort, 3);
  }

  /**
   * Set the state of a section of the left LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setLeftSection(int section, SectionState state) {
    m_leftLedController.setSectionState((byte) section, state);
  }

  /**
   * Set the state of a section of the right LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setRightSection(int section, SectionState state) {
    m_rightLedController.setSectionState((byte) section, state);
  }

  @Override
  public void close() throws Exception {
    m_leftLedController.close();
    m_rightLedController.close();
  }
}
