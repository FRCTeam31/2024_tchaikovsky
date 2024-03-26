package prime.control.LEDs.Patterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import prime.control.LEDs.Color;
import prime.control.LEDs.LEDEffect;

public abstract class LEDPattern {

  /**
   * The color of the pattern
   */
  public Color Color;

  /**
   * The effect of the pattern
   */
  public LEDEffect Effect;

  /**
   * The speed of the pattern in seconds per iteration
   */
  public double SpeedSeconds;

  /**
   * Whether the pattern is reversed. This is sometimes controlled by the pattern itself
   */
  public boolean Reversed;

  // State variables
  protected long Frame = 0;
  protected long LastFrameTime = 0;

  public LEDPattern(int r, int g, int b, LEDEffect effect, double speed, boolean reversed) {
    this.Color = new Color((byte) r, (byte) g, (byte) b);
    this.Effect = effect;
    this.SpeedSeconds = Math.max(speed, 0.005); // Minimum speed of 5ms
    this.Reversed = reversed;
  }

  public LEDPattern(Color color, LEDEffect effect, double speed, boolean reversed) {
    this.Color = color;
    this.Effect = effect;
    this.SpeedSeconds = Math.max(speed, 0.005); // Minimum speed of 5ms
    this.Reversed = reversed;
  }

  /**
   * Update the buffer with the pattern
   * @param startingIndex The starting index of the buffer to update
   * @param length The length of the buffer to update
   * @param buffer The buffer to update
   */
  public abstract void updateBuffer(int startingIndex, int length, AddressableLEDBuffer buffer);

  /**
   * Check if the pattern is the same as another pattern
   * @param other The other pattern to compare to
   */
  public boolean isSameAs(LEDPattern other) {
    return (
      this.Color.equals(other.Color) &&
      this.Effect == other.Effect &&
      this.SpeedSeconds == other.SpeedSeconds &&
      this.Reversed == other.Reversed
    );
  }

  /**
   * Check if the pattern is updatable based on the speed
   */
  protected boolean isUpdatable() {
    var currentTime = System.currentTimeMillis();

    return currentTime - LastFrameTime >= (SpeedSeconds * 1000);
  }
}
