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

    /**
     * Method for getting the inputs used by the Intake Subsystem (From Robot - To Code)
     * @return IntakeIOInputs
     */
    public IntakeIOInputs getInputs();

    /**
     * Method for setting the outputs from the Intake Subsystem (To Robot - From Code)
     * @param outputs
     */
    public void setOutputs(IntakeIOOutputs outputs);

    /**
     * Method that stops the angle motors for the Intake
     */
    public void StopMotors();

    /**
     * Method that stops the roller's motors in the Intake
     */
    public void StopRollers();

    /**
     * Method that runs the Intake rollers are a desired speed
     * @param speed
     */
    public void RunIntakeRollers(double speed);

    /**
     * Method that sets the speed of the Intake's angle motors
     * @param speed
     */
    public void SetAngleMotorSpeed(double speed);
}
