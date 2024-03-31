package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.LEDEffect;

public class PulsePattern extends LEDPattern {

  private static final int MIN_FRAME_COUNT = 70;
  private int _frameCount = MIN_FRAME_COUNT;

  /**
   * Create a new PulsePattern with a color and speed
   * @param color The color of the pattern
   * @param pulseSpeedSeconds The speed of the pattern in seconds per iteration
   */
  public PulsePattern(prime.control.LEDs.Color color, double pulseSpeedSeconds) {
    // Get frame count from pulseSpeedSeconds and minimum frame speed
    super(color, LEDEffect.Pulse, getFastestFrameSpeed(pulseSpeedSeconds), false);
    _frameCount = getMaxFrameCount(pulseSpeedSeconds);
  }

  /**
   * Create a new PulsePattern with an RGB color and speed
   * @param r The red value of the color
   * @param g The green value of the color
   * @param b The blue value of the color
   * @param pulseSpeedSeconds The speed of the pattern in seconds per iteration
   */
  public PulsePattern(int r, int g, int b, double pulseSpeedSeconds) {
    super(r, g, b, LEDEffect.Pulse, getFastestFrameSpeed(pulseSpeedSeconds), false);
    _frameCount = getMaxFrameCount(pulseSpeedSeconds);
  }

  /**
   * Get the fastest frame speed for the given pulse speed
   */
  private static double getFastestFrameSpeed(double pulseSpeedSeconds) {
    return pulseSpeedSeconds / getMaxFrameCount(pulseSpeedSeconds);
  }

  /**
   * Get the maximum frame count for the given pulse speed
   */
  private static int getMaxFrameCount(double pulseSpeedSeconds) {
    return Math.max((int) (pulseSpeedSeconds / MIN_FRAME_SPEED), MIN_FRAME_COUNT);
  }

  @Override
  public void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer) {
    if (isUpdatable()) {
      // calculate brightness of the color by the current Frame. Frame 0 is OFF, frame FRAME_COUNT is MAX_BRIGHTNESS
      var scalar = (float) Frame / _frameCount;
      var color = new prime.control.LEDs.Color(
        (byte) (Color.r * scalar),
        (byte) (Color.g * scalar),
        (byte) (Color.b * scalar)
      );

      // Set the color of each pixel in the buffer length
      for (int i = 0; i < length; i++) {
        buffer.setRGB(startingIndex + i, color.r, color.g, color.b);
      }

      // Increment the frame forward if not reversed, decrement if reversed
      Frame += !Reversed ? 1 : -1;

      if (Frame >= _frameCount) {
        // If the frame is at the end, set to reverse direction
        Reversed = true;
      } else if (Frame == 0) {
        // If the frame is at the start, set to forward direction
        Reversed = false;
      }

      LastFrameTime = System.currentTimeMillis();
    }
  }
}
