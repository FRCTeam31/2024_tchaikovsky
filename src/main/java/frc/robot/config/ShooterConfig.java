package frc.robot.config;

public class ShooterConfig {

  public int TalonFXCanID;
  public int VictorSPXCanID;
  public boolean TalonFXInverted;
  public boolean VictorSPXInverted;
  public int NoteDetectorDIOChannel;
  public int ElevationSolenoidForwardChannel;
  public int ElevationSolenoidReverseChannel;

  /**
   * Creates a new instance of ShooterConfig with default values
   */
  public ShooterConfig() {
    TalonFXCanID = 20;
    VictorSPXCanID = 19;
    TalonFXInverted = false;
    VictorSPXInverted = false;
    NoteDetectorDIOChannel = 7;
    ElevationSolenoidForwardChannel = 6;
    ElevationSolenoidReverseChannel = 7;
  }
}
