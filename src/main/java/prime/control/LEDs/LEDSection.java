package prime.control.LEDs;

public class LEDSection {

  public Color color;
  public LEDPattern pattern;
  public byte speed;
  public boolean directionReversed;

  public LEDSection(int r, int g, int b, LEDPattern pattern, int speed, boolean reversed) {
    this.color = new Color((byte) r, (byte) g, (byte) b);
    this.pattern = pattern;
    this.speed = (byte) speed;
    this.directionReversed = reversed;
  }

  public LEDSection(Color color, LEDPattern pattern, int speed, boolean reversed) {
    this.color = color;
    this.pattern = pattern;
    this.speed = (byte) speed;
    this.directionReversed = reversed;
  }

  public LEDSection(byte[] packet) {
    this(packet[1], packet[2], packet[3], LEDPattern.values()[packet[4]], packet[5], packet[6] == 1);
  }

  public byte[] toSectionPacket(byte section) {
    return new byte[] {
      section,
      this.color.r,
      this.color.g,
      this.color.b,
      (byte) this.pattern.ordinal(),
      this.speed,
      (byte) (this.directionReversed ? 1 : 0),
    };
  }

  public static LEDSection solidColor(Color color) {
    return new LEDSection(color, LEDPattern.Solid, 0, false);
  }

  public static LEDSection pulseColor(Color color, int speed) {
    return new LEDSection(color, LEDPattern.Pulse, speed, false);
  }

  public static LEDSection blinkColor(Color color, int speed) {
    return new LEDSection(color, LEDPattern.Blink, speed, false);
  }

  public static LEDSection raceColor(Color color, int speed, boolean reversed) {
    return new LEDSection(color, LEDPattern.Race, speed, reversed);
  }
}
