package frc.robot.config;

import edu.wpi.first.wpilibj.SerialPort.Port;

public class LEDConfig {

  public Port Port;
  public int SectionCount;

  public LEDConfig(Port port, int sectionCount) {
    Port = port;
    SectionCount = sectionCount;
  }
}
