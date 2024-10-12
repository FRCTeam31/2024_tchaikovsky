package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import prime.movers.LazyCANSparkMax;

public class IntakeIOReal implements IIntakeIO {

    private IIntakeIO intakeIO;
    private IntakeIOInputs intakeInputs = new IntakeIOInputs();
    private IntakeIOOutputs intakeOutputs = new IntakeIOOutputs();

    private DigitalInput m_topLimitSwitch;
    private DigitalInput m_bottomLimitSwitch;

    private LazyCANSparkMax m_rollers;
    private LazyCANSparkMax m_angleLeft;
    private LazyCANSparkMax m_angleRight;

    private PIDController m_anglePid;

    public IntakeIOReal() {


    }

    @Override
    public IntakeIOInputs getInputs() {
       var inputs = new IntakeIOInputs();

       inputs.m_angleRightPosition = m_angleRight.getEncoder().getPosition();
       inputs.m_angleLeftPosition = m_angleLeft.getEncoder().getPosition();

       inputs.m_angleLeftState = m_angleLeft.get();
       inputs.m_angleRightState = m_angleRight.get();
       inputs.m_rollersState = m_rollers.get();

       inputs.m_topLimitSwitchState = m_topLimitSwitch.get();
       inputs.m_bottomLimitSwitchState = m_bottomLimitSwitch.get();

       inputs.m_anglePidOutput = m_anglePid.calculate(inputs.m_angleRightPosition, intakeOutputs.m_angleToggledIn ? intakeOutputs.m_angleStartPoint : (intakeOutputs.m_angleStartPoint - IntakeSubsystem.VMap.POSITION_DELTA));

       return inputs;
    }

    @Override
    public void setOutputs(IntakeIOOutputs outputs) {
        var inputs = getInputs();
    }

    @Override
    public void StopMotors() {
        m_angleLeft.stopMotor();
        m_angleRight.stopMotor();
    }

    @Override
    public void StopRollers() {
        m_rollers.stopMotor();
    }

    @Override
    public void RunIntakeRollers(double speed) {
        m_rollers.set(speed);
    }

    @Override
    public void SetAngleMotorSpeed(double speed) {
        m_angleLeft.set(-speed);
        m_angleRight.set(speed);
    }
    
}
