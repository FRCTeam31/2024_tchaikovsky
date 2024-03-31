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
  public ChasePattern(prime.control.LEDs.Color color, double speed, boolean reversed) {
    super(color, LEDEffect.Chase, speed, reversed);
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
    var frameSpeedS = EffectSpeedSeconds / totalFrameCount;
    var currentTime = System.currentTimeMillis();

    if (currentTime - LastFrameTime >= (frameSpeedS * 1000)) {
      if (!Reversed) {
        // Set the chase group
        var chaseGroupFirst = startingIndex + Frame;
        var chaseGroupLast = chaseGroupFirst - CHASE_LENGTH;
        for (long i = chaseGroupLast; i <= chaseGroupFirst; i++) {
          if (i >= startingIndex && i < startingIndex + length) {
            buffer.setRGB((int) i, Color.r, Color.g, Color.b);
          }
        }

        // Set the fading trail
        var fadeTrailFirst = chaseGroupLast - 1;
        var fadeTrailLast = fadeTrailFirst - FADE_LENGTH;
        for (long i = fadeTrailLast; i <= fadeTrailFirst; i++) {
          if (i >= startingIndex && i < startingIndex + length) {
            var brightnessScalar = (float) i / FADE_LENGTH;
            buffer.setRGB(
              (int) i,
              (byte) (Color.r * brightnessScalar),
              (byte) (Color.g * brightnessScalar),
              (byte) (Color.b * brightnessScalar)
            );
          }
        }

        Frame = Frame + 1;
        if (Frame >= totalFrameCount) {
          Frame = 0;
        }
      } else {
        // Set the chase group
        var chaseGroupFirst = startingIndex + length - Frame;
        var chaseGroupLast = chaseGroupFirst + CHASE_LENGTH;
        for (long i = chaseGroupLast; i >= chaseGroupFirst; i--) {
          if (i >= startingIndex && i < startingIndex + length) {
            buffer.setRGB((int) i, Color.r, Color.g, Color.b);
          }
        }

        // Set the fading trail
        var fadeTrailFirst = chaseGroupLast + 1;
        var fadeTrailLast = fadeTrailFirst + FADE_LENGTH;
        for (long i = fadeTrailLast; i >= fadeTrailFirst; i--) {
          if (i >= startingIndex && i < startingIndex + length) {
            var brightnessScalar = (float) (length - i) / FADE_LENGTH;
            buffer.setRGB(
              (int) i,
              (byte) (Color.r * brightnessScalar),
              (byte) (Color.g * brightnessScalar),
              (byte) (Color.b * brightnessScalar)
            );
          }
        }

        Frame = Frame - 1;
        if (Frame <= 0) {
          Frame = totalFrameCount;
        }
      }

      LastFrameTime = System.currentTimeMillis();
    }
  }
}
