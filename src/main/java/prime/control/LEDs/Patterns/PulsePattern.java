package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.LEDEffect;

public class PulsePattern extends LEDPattern {

  private static final double FRAME_COUNT = 50;

  /**
   * Create a new PulsePattern with a color and speed
   * @param color The color of the pattern
   * @param speed The speed of the pattern in seconds per iteration
   */
  public PulsePattern(prime.control.LEDs.Color color, double speed) {
    super(color, LEDEffect.Pulse, speed / FRAME_COUNT, false);
  }

  /**
   * Create a new PulsePattern with an RGB color and speed
   * @param r The red value of the color
   * @param g The green value of the color
   * @param b The blue value of the color
   * @param speed The speed of the pattern in seconds per iteration
   */
  public PulsePattern(int r, int g, int b, double speed) {
    super(r, g, b, LEDEffect.Pulse, speed / FRAME_COUNT, false);
  }

  @Override
  public void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer) {
    if (isUpdatable()) {
      // calculate brightness of the color by the current Frame. Frame 0 is OFF, frame FRAME_COUNT is MAX_BRIGHTNESS
      var scalar = (float) Frame / FRAME_COUNT;
      var color = new prime.control.LEDs.Color(
        (byte) (Color.r * scalar),
        (byte) (Color.g * scalar),
        (byte) (Color.b * scalar)
      );

      for (int i = 0; i < length; i++) {
        buffer.setRGB(startingIndex + i, color.r, color.g, color.b);
      }

      // Increment the frame forward if !reversed, decrement if Reversed
      Frame = Reversed ? Frame - 1 : Frame + 1;

      // If the frame is at the start, set to forward direction
      // If the frame is at the end, set to reverse direction
      if (Frame == FRAME_COUNT) {
        Reversed = true;
      } else if (Frame == 0) {
        Reversed = false;
      }

      LastFrameTime = System.currentTimeMillis();
    }
  }
}
