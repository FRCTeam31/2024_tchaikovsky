package prime.control.LEDs;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class PrimeLEDController implements AutoCloseable {

  private SerialPort m_serial;
  private LEDSection[] m_lastPackets;

  public PrimeLEDController(Port port, int sectionCount) {
    m_serial = new SerialPort(115200, port);
    m_lastPackets = new LEDSection[sectionCount];
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
  public void setSectionState(byte section, LEDSection state) {
    if (section < 0 || section > 2) {
      System.out.println("Invalid section number");
      return;
    }

    var packet = state.toSectionPacket(section);

    // Write the data packet to the controller
    System.out.println("Writing " + packet.length + " bytes to LEDs for section " + section);
    m_serial.write(packet, packet.length);
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
    m_serial.close();
  }
}
