package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.LEDConfig;
import prime.control.LEDs.PrimeLEDController;

public class LEDStrips extends SubsystemBase implements AutoCloseable {

  private LEDConfig m_config;
  private PrimeLEDController m_leftLedController;
  private PrimeLEDController m_rightLedController;

  public LEDStrips(LEDConfig config) {
    m_config = config;
    m_leftLedController =
      new PrimeLEDController(m_config.LEDStripLeftAddress, 3);
    m_rightLedController =
      new PrimeLEDController(m_config.LEDStripRightAddress, 3);
  }

  @Override
  public void close() throws Exception {
    m_leftLedController.close();
    m_rightLedController.close();
  }
}
