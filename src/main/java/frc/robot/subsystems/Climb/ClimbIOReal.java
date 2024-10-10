package frc.robot.subsystems.Climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.DriverDashboard;

public class ClimbIOReal implements IClimbIO {

    private IClimbIO climbIO;
    private ClimbIOInputs climbInputs = new ClimbIOInputs();
    private ClimbIOOutputs climbOutputs = new ClimbIOOutputs();

    // Motors
    private VictorSPX m_leftVictorSPX;
    private VictorSPX m_rightVictorSPX;

    // Limit Switches
    private DigitalInput m_leftLimitSwitch;
    private DigitalInput m_rightLimitSwitch;

    // Clutch Solenoids
    private DoubleSolenoid m_clutchSolenoidLeft;
    private DoubleSolenoid m_clutchSolenoidRight;

    public ClimbIOReal() {

        m_leftVictorSPX = new VictorSPX(ClimbSubsystem.VMap.VICTORSPX_LEFT_CAN_ID);
        m_leftVictorSPX.configFactoryDefault();
        m_leftVictorSPX.setInverted(ClimbSubsystem.VMap.LEFT_INVERTED);
        m_leftVictorSPX.setNeutralMode(NeutralMode.Brake);
        m_leftVictorSPX.configOpenloopRamp(0.5);

        m_rightVictorSPX = new VictorSPX(ClimbSubsystem.VMap.VICTORSPX_RIGHT_CAN_ID);
        m_rightVictorSPX.configFactoryDefault();
        m_rightVictorSPX.setInverted(ClimbSubsystem.VMap.RIGHT_INVERTED);
        m_rightVictorSPX.setNeutralMode(NeutralMode.Brake);
        m_rightVictorSPX.configOpenloopRamp(0.5);

        m_leftLimitSwitch = new DigitalInput(ClimbSubsystem.VMap.LEFT_LIMIT_SWITCH_DIO_CHANNEL);
        m_rightLimitSwitch = new DigitalInput(ClimbSubsystem.VMap.RIGHT_LIMIT_SWITCH_DIO_CHANNEL);

        m_clutchSolenoidLeft =
          new DoubleSolenoid(
            30,
            PneumaticsModuleType.REVPH,
            ClimbSubsystem.VMap.LEFT_SOLENOID_FORWARD_CHANNEL,
            ClimbSubsystem.VMap.LEFT_SOLENOID_REVERSE_CHANNEL
          );
        m_clutchSolenoidRight =
          new DoubleSolenoid(
            30,
            PneumaticsModuleType.REVPH,
            ClimbSubsystem.VMap.RIGHT_SOLENOID_FORWARD_CHANNEL,
            ClimbSubsystem.VMap.LEFT_SOLENOID_REVERSE_CHANNEL
          );
    }

    @Override
    public ClimbIOInputs getInputs() {
        var inputs = new ClimbIOInputs();

        inputs.m_leftLimitSwitchState = m_leftLimitSwitch.get();
        inputs.m_rightLimitSwitchState = m_rightLimitSwitch.get();

        inputs.m_leftVictorSPXOutputPercent = m_leftVictorSPX.getMotorOutputPercent();
        inputs.m_rightVictorSPXOutputPercent = m_rightVictorSPX.getMotorOutputPercent();

        return inputs;
    }

    @Override
    public void setOutputs(ClimbIOOutputs outputs) {
        
    }

    @Override
    public void MoveArm(Side side, Direction direction, double speed) {
        if (direction == Direction.kRaise) {

            if (side == Side.kLeft && !m_leftLimitSwitch.get()) {
                m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, ClimbSubsystem.VMap.CLIMBER_UP_SPEED);
            }

            if (side == Side.kRight && !m_rightLimitSwitch.get()) {
                m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, ClimbSubsystem.VMap.CLIMBER_UP_SPEED);
            }
        }

        if (direction == Direction.kLower) {

            if (side == Side.kLeft) {
              m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
            }

            if (side == Side.kRight) {
              m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, -speed);
            }
        }
    }

    @Override
    public void StopArm(Side side) {
        if (side == Side.kLeft) {
            m_leftVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
        }

        if (side == Side.kRight) {
            m_rightVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void SetClutch(Side side, boolean engaged) {
        if (side == Side.kLeft) {
          m_clutchSolenoidLeft.set(engaged ? Value.kForward : Value.kReverse);
        }

        if (side == Side.kRight) {
          m_clutchSolenoidRight.set(engaged ? Value.kForward : Value.kReverse);
        }
    }
    
}
