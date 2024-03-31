package frc.robot.config;

public class ClimbersConfig {

  public int VictorSPXLeftCanID;
  public int VictorSPXRightCanID;

  public boolean LeftInverted;
  public boolean RightInverted;

  public double ClimberUpSpeed;
  public double ClimberDownSpeed;

  public int LeftLimitSwitchDIOChannel;
  public int RightLimitSwitchDIOChannel;

  public int LeftSolenoidForwardChannel;
  public int LeftSolenoidReverseChannel;
  public int RightSolenoidForwardChannel;
  public int RightSolenoidReverseChannel;

  /**
   * Creates a new instance of ClimbersConfig with default values
   */
  public ClimbersConfig() {
    VictorSPXLeftCanID = 18;
    VictorSPXRightCanID = 17;
    LeftInverted = true;
    RightInverted = true;
    ClimberUpSpeed = 0.5;
    ClimberDownSpeed = -1;
    LeftLimitSwitchDIOChannel = 2;
    RightLimitSwitchDIOChannel = 3;
    LeftSolenoidForwardChannel = 8;
    LeftSolenoidReverseChannel = 9;
    RightSolenoidForwardChannel = 12;
    RightSolenoidReverseChannel = 11;
  }
}
