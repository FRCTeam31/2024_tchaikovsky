package frc.robot.config;

public class LEDConfig {

  public int PwmPort;
  public int SectionCount;
  public int PixelsPerSection;

  public LEDConfig(int pwmPort, int sectionCount, int pixelsPerSection) {
    PwmPort = pwmPort;
    SectionCount = sectionCount;
    PixelsPerSection = pixelsPerSection;
  }
}
