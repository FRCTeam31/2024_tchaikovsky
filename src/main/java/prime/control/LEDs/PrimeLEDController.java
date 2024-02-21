package prime.control.LEDs;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class PrimeLEDController implements AutoCloseable {

  private I2C m_i2c;
  private SectionState[] m_lastPackets;

  public PrimeLEDController(int deviceI2CAddress, int sectionCount) {
    m_i2c = new I2C(Port.kOnboard, deviceI2CAddress);
    m_lastPackets = new SectionState[sectionCount];
  }

  /**
   * Write a packet to the LED controller containing the new state of the LEDs
   * @param sectionNum
   * @param r
   * @param g
   * @param b
   * @param brightness
   * @param pattern
   * @param speed
   */
  public void setSectionState(byte section, SectionState state) {
    if (section < 0 || section > 3) {
      System.out.println("Invalid section number");
      return;
    }

    var packet = state.toSectionPacket(section);

    // Write the data packet to the controller
    m_i2c.writeBulk(packet, packet.length);
  }

  /**
   * Write a Color packet to the LED controller
   * @param section
   * @param newColor
   */
  public void setColor(byte section, Color newColor) {
    m_lastPackets[section].color = newColor;

    setSectionState(section, m_lastPackets[section]);
  }

  /**
   * Write a Pattern packet to the LED controller
   * @param section
   * @param newPattern
   */
  public void setPattern(byte section, LEDPattern newPattern) {
    m_lastPackets[section].pattern = newPattern;

    setSectionState(section, m_lastPackets[section]);
  }

  /**
   * Write a Speed packet to the LED controller
   * @param section
   * @param newSpeed
   */
  public void setSpeed(byte section, byte newSpeed) {
    m_lastPackets[section].speed = newSpeed;

    setSectionState(section, m_lastPackets[section]);
  }

  /**
   * Write a Direction packet to the LED controller
   * @param section
   * @param speed
   */
  public void setDirection(byte section, boolean newDirectionReversed) {
    m_lastPackets[section].directionReversed = newDirectionReversed;

    setSectionState(section, m_lastPackets[section]);
  }

  @Override
  public void close() throws Exception {
    m_i2c.close();
  }
}
