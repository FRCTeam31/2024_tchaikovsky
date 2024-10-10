package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IIntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

        public boolean m_topLimitSwitchState;
        public boolean m_bottomLimitSwitchState;

        public double m_angleLeftPosition;
        public double m_angleRightPosition;

        public double m_angleLeftState;
        public double m_angleRightState;
        public double m_rollersState;

        public double m_anglePidOutput;

    }

    @AutoLog
    public static class IntakeIOOutputs { 
        
        public boolean m_angleToggledIn = false;
        public double m_angleStartPoint;

    }

    public IntakeIOInputs getInputs();

    public void setOutputs(IntakeIOOutputs outputs);

    public void StopMotors();

    public void StopRollers();

    public void RunIntakeRollers(double speed);

    public void SetAngleMotorSpeed(double speed);
}
