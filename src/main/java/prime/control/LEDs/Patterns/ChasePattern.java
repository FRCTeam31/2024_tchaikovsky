package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.LEDEffect;

public class ChasePattern extends LEDPattern {

  private static final int CHASE_LENGTH = 4; // Length of the chasing group
  private static final int FADE_LENGTH = 8; // Length of the fading tail

  /**
   * Create a new ChasePattern with a color and speed
   *
   * @param color The color of the pattern
   * @param speed The speed of the pattern in seconds per iteration
   */
  public ChasePattern(prime.control.LEDs.Color color, double speed) {
    super(color, LEDEffect.Chase, speed, false);
  }

  /**
   * Create a new ChasePattern with an RGB color and speed
   *
   * @param r     The red value of the color
   * @param g     The green value of the color
   * @param b     The blue value of the color
   * @param speed The speed of the pattern in seconds per iteration
   */
  public ChasePattern(int r, int g, int b, double speed) {
    super(r, g, b, LEDEffect.Chase, speed, false);
  }

  @Override
  public void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer) {
    var totalFrameCount = length + CHASE_LENGTH + FADE_LENGTH;
    var frameSpeedS = SpeedSeconds / totalFrameCount;
    var currentTime = System.currentTimeMillis();

    if (currentTime - LastFrameTime >= (frameSpeedS * 1000)) {
      // TODO: Implement ChasePattern

      LastFrameTime = System.currentTimeMillis();
    }
  }
}
