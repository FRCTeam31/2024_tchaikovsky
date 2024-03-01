package frc.robot.config;

import edu.wpi.first.wpilibj.SerialPort.Port;

public class LEDConfig {

  public Port LeftPort;
  public Port RightPort;

  public LEDConfig(Port leftPort, Port rightPort) {
    LeftPort = leftPort;
    RightPort = rightPort;
  }
}
