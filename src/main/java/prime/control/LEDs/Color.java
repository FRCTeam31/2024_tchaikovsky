package prime.control.LEDs;

public class Color {

  public byte r;
  public byte g;
  public byte b;

  public Color(byte r, byte g, byte b) {
    this.r = r;
    this.g = g;
    this.b = b;
  }

  // Predefined static colors
  public static final Color OFF = new Color((byte) 0, (byte) 0, (byte) 0);

  public static final Color RED = new Color((byte) 255, (byte) 0, (byte) 0);

  public static final Color GREEN = new Color((byte) 0, (byte) 255, (byte) 0);

  public static final Color BLUE = new Color((byte) 0, (byte) 0, (byte) 255);

  public static final Color WHITE = new Color((byte) 255, (byte) 255, (byte) 255);

  public static final Color YELLOW = new Color((byte) 255, (byte) 255, (byte) 0);

  public static final Color CYAN = new Color((byte) 0, (byte) 255, (byte) 255);

  public static final Color MAGENTA = new Color((byte) 255, (byte) 0, (byte) 255);

  public static final Color ORANGE = new Color((byte) 255, (byte) 165, (byte) 0);

  public static final Color PINK = new Color((byte) 255, (byte) 192, (byte) 203);

  public static final Color PURPLE = new Color((byte) 128, (byte) 0, (byte) 128);

  public static final Color LIME = new Color((byte) 0, (byte) 255, (byte) 0);

  public static final Color TEAL = new Color((byte) 0, (byte) 128, (byte) 128);

  public static final Color AQUA = new Color((byte) 0, (byte) 255, (byte) 255);

  public static final Color VIOLET = new Color((byte) 238, (byte) 130, (byte) 238);

  public static final Color BROWN = new Color((byte) 165, (byte) 42, (byte) 42);

  public static final Color TAN = new Color((byte) 210, (byte) 180, (byte) 140);

  public static final Color BEIGE = new Color((byte) 245, (byte) 245, (byte) 220);

  public static final Color MAROON = new Color((byte) 128, (byte) 0, (byte) 0);

  public static final Color OLIVE = new Color((byte) 128, (byte) 128, (byte) 0);

  public static final Color NAVY = new Color((byte) 0, (byte) 0, (byte) 128);
}
