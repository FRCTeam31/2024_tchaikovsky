package frc.robot.config;

import edu.wpi.first.wpilibj.SerialPort.Port;

public class LEDConfig {

  public Port Port;

  public LEDConfig(Port port) {
    Port = port;
  }
}
