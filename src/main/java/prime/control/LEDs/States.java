package prime.control.LEDs;

public class States {

  public static final LEDSection OFF = new LEDSection(0, 0, 0, LEDPattern.Solid, 0, false);

  public static LEDSection SOLID(Color color) {
    return new LEDSection(color, LEDPattern.Solid, 0, false);
  }

  public static LEDSection BLINK(Color color, int speed) {
    return new LEDSection(color, LEDPattern.Blink, speed, false);
  }

  public static LEDSection PULSE(Color color, int speed, boolean reversed) {
    return new LEDSection(color, LEDPattern.Pulse, speed, reversed);
  }

  public static LEDSection RACE(Color color, int speed, boolean reversed) {
    return new LEDSection(color, LEDPattern.Race, speed, reversed);
  }
}
