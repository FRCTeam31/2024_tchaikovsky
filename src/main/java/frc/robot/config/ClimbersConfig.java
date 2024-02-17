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

  public int LeftServoChannel;
  public int RightServoChannel;
  public int ServoLockAngle;
  public int ServoUnlockAngle;

  public ClimbersConfig(
    int victorSPXLeftCanID,
    int victorSPXRightCanID,
    boolean leftInverted,
    boolean rightInverted,
    double climberUpSpeed,
    int climberDownSpeed,
    int leftLimitSwitchDIOChannel,
    int rightLimitSwitchDIOChannel,
    int leftServoChannel,
    int rightServoChannel,
    int servoLockAngle,
    int servoUnlockAngle
  ) {
    VictorSPXLeftCanID = victorSPXLeftCanID;
    VictorSPXRightCanID = victorSPXRightCanID;
    LeftInverted = leftInverted;
    RightInverted = rightInverted;
    ClimberUpSpeed = climberUpSpeed;
    ClimberDownSpeed = climberDownSpeed;
    LeftLimitSwitchDIOChannel = leftLimitSwitchDIOChannel;
    RightLimitSwitchDIOChannel = rightLimitSwitchDIOChannel;
    LeftServoChannel = leftServoChannel;
    RightServoChannel = rightServoChannel;
    ServoLockAngle = servoLockAngle;
    ServoUnlockAngle = servoUnlockAngle;
  }
}
