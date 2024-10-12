package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.PwmLEDs;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.BlinkPattern;
import prime.control.LEDs.Patterns.ChasePattern;

public class ShooterIOReal implements IShooterIO{
    private PwmLEDs m_leds;
    private TalonFX m_talonFX;
    private VictorSPX m_victorSPX;
    private DoubleSolenoid m_elevationSolenoid;
    private DigitalInput m_noteDetector;

    private IShooterIO shooterIO;
    private ShooterIOInputs shooterInputs = new ShooterIOInputs();
    private ShooterIOOutputs shooterOutputs = new ShooterIOOutputs();

    public ShooterIOReal(PwmLEDs leds) {
        m_leds = leds;

        m_talonFX = new TalonFX(ShooterSubsystem.VMap.TALONFX_CAN_ID);
        m_talonFX.getConfigurator().apply(new TalonFXConfiguration());
        m_talonFX.setInverted(true);
        m_talonFX.setNeutralMode(NeutralModeValue.Brake);

        m_victorSPX = new VictorSPX(ShooterSubsystem.VMap.VICTORSPX_CAN_ID);
        m_victorSPX.configFactoryDefault();
        m_victorSPX.setNeutralMode(NeutralMode.Brake);

        m_elevationSolenoid =
          new DoubleSolenoid(
            30,
            PneumaticsModuleType.REVPH,
            ShooterSubsystem.VMap.ELEVATION_SOLENOID_FORWARD_CHANNEL,
            ShooterSubsystem.VMap.ELEVATION_SOLENOID_REVERSE_CHANNEL
          );

        m_noteDetector = new DigitalInput(ShooterSubsystem.VMap.NOTE_DETECTOR_DIO_CHANNEL);
    }

    @Override
    public ShooterIOInputs getInputs() {
        var inputs = new ShooterIOInputs();

        inputs.m_noteDetectorState = m_noteDetector.get();

        inputs.m_talonFXVelocity = m_talonFX.getVelocity().getValueAsDouble();
        inputs.m_talonFXState = m_talonFX.get();

        inputs.m_victorSPXOutputPercent = m_victorSPX.getMotorOutputPercent();
        
        inputs.m_elevationSolenoidState = m_elevationSolenoid.get();

        return inputs;
    }

    @Override
    public void setOutputs(ShooterIOOutputs outputs) {
        var inputs = new ShooterIOInputs();

        // if (outputs.m_stopTalonFX) {
        //     m_talonFX.stopMotor();
        //     outputs.m_stopTalonFX = false;
        // } else {
        //     m_talonFX.set(outputs.m_talonFXSpeed);
        // }

        // if (outputs.m_stopVictorSPX) {
        //     m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);
        //     outputs.m_stopVictorSPX = false;
        // } else {
        //     m_victorSPX.set(VictorSPXControlMode.PercentOutput, outputs.m_victorSPXSpeed);
        // }

        //  m_elevationSolenoid.set(outputs.m_elevationSolenoidValue);

    }

    @Override
    public void ResetLEDs() {
        m_leds.restorePersistentStripPattern();
    }

    @Override
    public void RunNoteDetectedLEDPattern() {
        m_leds.setStripTemporaryPattern(new BlinkPattern(prime.control.LEDs.Color.ORANGE, 0.2));
    }

    @Override
    public void RunShootingNoteLEDPattern() {
        m_leds.setStripTemporaryPattern(new ChasePattern(Color.GREEN, 0.25, !shooterInputs.m_noteDetectorState));
    }

    @Override
    public void StopMotors() {
        m_talonFX.stopMotor();
        m_victorSPX.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public void RunShooter(double speed) {
        m_talonFX.set(speed);
        m_victorSPX.set(VictorSPXControlMode.PercentOutput, speed * 3);
    }

    @Override
    public void RunGreenWheel(double speed) {
        m_victorSPX.set(VictorSPXControlMode.PercentOutput, speed);
    }

    @Override
    public void SetElevator(Value value) {
        m_elevationSolenoid.set(value);
    }

}