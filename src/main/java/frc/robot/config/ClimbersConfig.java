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

  public ClimbersConfig(
    int victorSPXLeftCanID,
    int victorSPXRightCanID,
    boolean leftInverted,
    boolean rightInverted,
    double climberUpSpeed,
    int climberDownSpeed,
    int leftLimitSwitchDIOChannel,
    int rightLimitSwitchDIOChannel,
    int leftSolenoidForwardChannel,
    int leftSolenoidReverseChannel,
    int rightSolenoidForwardChannel,
    int rightSolenoidReverseChannel
  ) {
    VictorSPXLeftCanID = victorSPXLeftCanID;
    VictorSPXRightCanID = victorSPXRightCanID;

    LeftInverted = leftInverted;
    RightInverted = rightInverted;

    ClimberUpSpeed = climberUpSpeed;
    ClimberDownSpeed = climberDownSpeed;

    LeftLimitSwitchDIOChannel = leftLimitSwitchDIOChannel;
    RightLimitSwitchDIOChannel = rightLimitSwitchDIOChannel;

    LeftSolenoidForwardChannel = leftSolenoidForwardChannel;
    LeftSolenoidReverseChannel = leftSolenoidReverseChannel;
    RightSolenoidForwardChannel = rightSolenoidForwardChannel;
    RightSolenoidReverseChannel = rightSolenoidReverseChannel;
  }
}
