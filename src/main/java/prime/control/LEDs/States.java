package prime.control.LEDs;

public class States {

  public static final SectionState OFF = new SectionState(
    0,
    0,
    0,
    LEDPattern.Solid,
    0,
    false
  );

  public static SectionState SOLID(Color color) {
    return new SectionState(color, LEDPattern.Solid, 0, false);
  }

  public static SectionState BLINK(Color color, int speed) {
    return new SectionState(color, LEDPattern.Blink, speed, false);
  }

  public static SectionState PULSE(Color color, int speed, boolean reversed) {
    return new SectionState(color, LEDPattern.Pulse, speed, reversed);
  }

  public static SectionState RACE(Color color, int speed, boolean reversed) {
    return new SectionState(color, LEDPattern.Race, speed, reversed);
  }
}
