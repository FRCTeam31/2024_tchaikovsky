package frc.robot.config;

public class LEDConfig {

  public int PwmPort;
  public int SectionCount;
  public int PixelsPerSection;

  /**
   * Creates a new instance of LEDConfig with default values
   */
  public LEDConfig() {
    PwmPort = 5;
    SectionCount = 1;
    PixelsPerSection = 78;
  }
}
