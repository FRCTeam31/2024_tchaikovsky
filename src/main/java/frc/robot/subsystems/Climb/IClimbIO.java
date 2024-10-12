package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.AutoLog;

public interface IClimbIO {
    @AutoLog
    public static class ClimbIOInputs {

        public boolean m_leftLimitSwitchState;
        public boolean m_rightLimitSwitchState;

        public double m_leftVictorSPXOutputPercent = 0;
        public double m_rightVictorSPXOutputPercent = 0;
        
    }

    @AutoLog
    public static class ClimbIOOutputs {

    }

    /**
     * Used to determind what climber side should move
     */
    public enum Side {
        kLeft,
        kRight
    }

    /**
     * Used to determind the direction the climber arm should move
     */
    public enum Direction {
        kRaise,
        kLower
    }

    /**
     * Method for getting the inputs used by the Climb Subsystem (From Robot - To Code)
     * @return ClimberIOInputs
     */
    public ClimbIOInputs getInputs();

    /**
     * Method for setting the outputs from the Climb Subsystem (To Robot - From Code)
     * @param outputs
     */
    public void setOutputs(ClimbIOOutputs outputs);

    /**
     * Method that controls the movement of the climb arms based on a side, direction, and speed
     * @param side The side that should move
     * @param direction The direction the arm should move
     * @param speed Only implemented for when the arm is lowering (Thats how it was previously ¯\_(ツ)_/¯) - the travel speed of the climb arm
     */
    public void MoveArm(Side side, Direction direction, double speed);

    /**
     * Method that stops the desired climb arm
     * @param side The side to stop
     */
    public void StopArm(Side side);

    /**
     * Method to engage and disengage the clutch on the desired climb arm
     * @param side The side to set the clutch on
     * @param engaged The state the clutch should be set to
     */
    public void SetClutch(Side side, boolean engaged);
}
