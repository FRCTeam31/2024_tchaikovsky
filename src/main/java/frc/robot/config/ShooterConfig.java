package frc.robot.config;

public class ShooterConfig {

  public int TalonFXCanID;
  public int VictorSPXCanID;
  public boolean TalonFXInverted;
  public boolean VictorSPXInverted;
  public int NoteDetectorDIOChannel;
  public int ElevationSolenoidForwardChannel;
  public int ElevationSolenoidReverseChannel;

  public ShooterConfig(
    int shooterTalonFXCanID,
    int shooterVictorSPXCanID,
    boolean shooterTalonFXInverted,
    boolean shooterVictorSPXInverted,
    int noteDetectorDIOChannel,
    int elevationSolenoidForwardChannel,
    int elevationSolenoidReverseChannel
  ) {
    TalonFXCanID = shooterTalonFXCanID;
    VictorSPXCanID = shooterVictorSPXCanID;
    TalonFXInverted = shooterTalonFXInverted;
    VictorSPXInverted = shooterVictorSPXInverted;
    NoteDetectorDIOChannel = noteDetectorDIOChannel;
    ElevationSolenoidForwardChannel = elevationSolenoidForwardChannel;
    ElevationSolenoidReverseChannel = elevationSolenoidReverseChannel;
  }
}
