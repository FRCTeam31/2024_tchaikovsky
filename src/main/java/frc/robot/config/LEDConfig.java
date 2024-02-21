package frc.robot.config;

public class LEDConfig {

  public int LEDStripLeftAddress;
  public int LEDStripRightAddress;

  public LEDConfig(int leftAddress, int rightAddress) {
    LEDStripLeftAddress = leftAddress;
    LEDStripRightAddress = rightAddress;
  }
}
