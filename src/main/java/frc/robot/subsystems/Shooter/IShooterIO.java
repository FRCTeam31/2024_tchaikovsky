package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs{

        public boolean m_noteDetectorState = false;

        public double m_talonFXVelocity = 0;
        public double m_talonFXState;

        public double m_victorSPXOutputPercent = 0;

        public Value m_elevationSolenoidState;

    }

    @AutoLog
    public static class ShooterIOOutputs{ 

        // public double m_talonFXSpeed = 0;
        // public double m_victorSPXSpeed = 0;

        // public boolean m_stopTalonFX = false;
        // public boolean m_stopVictorSPX = false;

        // public Value m_elevationSolenoidValue;

    }

    public ShooterIOInputs getInputs();

    public void setOutputs(ShooterIOOutputs outputs);

    public void ResetLEDs();

    public void RunNoteDetectedLEDPattern();

    public void RunShootingNoteLEDPattern();

    public void StopMotors();

    public void RunShooter(double speed);

    public void RunGreenWheel(double speed);

    public void SetElevator(Value value);
}