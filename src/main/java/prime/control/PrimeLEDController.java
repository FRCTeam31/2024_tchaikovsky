package prime.control;

import edu.wpi.first.wpilibj.SerialPort;

public class PrimeLEDController {

  public class Color {

    public byte r;
    public byte g;
    public byte b;

    public Color(byte r, byte g, byte b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public enum LEDPattern {
    Solid,
    Blink,
    RaceForward,
    RaceBackward,
    Pulse,
  }

  private SerialPort m_serialPort;
  private byte[][] m_lastPackets;

  public PrimeLEDController(int sectionCount) {
    m_serialPort = new SerialPort(115200, SerialPort.Port.kUSB);
    m_lastPackets = new byte[sectionCount][];
  }

  /**
   * Write a packet to the LED controller containing the new state of the LEDs
   * @param stripNum
   * @param r
   * @param g
   * @param b
   * @param brightness
   * @param pattern
   * @param speed
   */
  public void setState(
    byte stripNum,
    byte r,
    byte g,
    byte b,
    byte brightness,
    byte pattern,
    byte speed,
    byte direction
  ) {
    if (stripNum < 0 || stripNum > 3) {
      System.out.println("Invalid strip number");
      return;
    }

    var packet = new byte[] {
      stripNum,
      r,
      g,
      b,
      brightness,
      pattern,
      speed,
      direction,
    };

    // Write the data packet to the controller
    m_serialPort.write(packet, packet.length);

    // Verify that the controller acknowledged the packet
    if (m_serialPort.readString(3) != "ACK") {
      System.out.println("LED Controller did not acknowledge SetState command");
    } else {
      // Save the last packet sent for this strip
      m_lastPackets[stripNum] = packet;
    }
  }

  /**
   * Write a Color packet to the LED controller
   * @param strip
   * @param r
   * @param g
   * @param b
   */
  public void setColor(byte strip, byte r, byte g, byte b) {
    setState(
      strip,
      r,
      g,
      b,
      m_lastPackets[strip][4],
      m_lastPackets[strip][5],
      m_lastPackets[strip][6],
      m_lastPackets[strip][7]
    );
  }

  /**
   * Write a Color packet to the LED controller
   * @param strip
   * @param color
   */
  public void setColor(byte strip, Color color) {
    setColor(strip, color.r, color.g, color.b);
  }

  /**
   * Write a Brightness packet to the LED controller
   * @param strip
   * @param brightness
   */
  public void setBrightness(byte strip, byte brightness) {
    setState(
      strip,
      m_lastPackets[strip][1],
      m_lastPackets[strip][2],
      m_lastPackets[strip][3],
      brightness,
      m_lastPackets[strip][5],
      m_lastPackets[strip][6],
      m_lastPackets[strip][7]
    );
  }

  /**
   * Write a Pattern packet to the LED controller
   * @param strip
   * @param pattern
   */
  public void setPattern(byte strip, LEDPattern pattern) {
    setState(
      strip,
      m_lastPackets[strip][1],
      m_lastPackets[strip][2],
      m_lastPackets[strip][3],
      m_lastPackets[strip][4],
      (byte) pattern.ordinal(),
      m_lastPackets[strip][6],
      m_lastPackets[strip][7]
    );
  }

  /**
   * Write a Speed packet to the LED controller
   * @param strip
   * @param speed
   */
  public void setSpeed(byte strip, byte speed) {
    setState(
      strip,
      m_lastPackets[strip][1],
      m_lastPackets[strip][2],
      m_lastPackets[strip][3],
      m_lastPackets[strip][4],
      m_lastPackets[strip][5],
      speed,
      m_lastPackets[strip][7]
    );
  }
}
