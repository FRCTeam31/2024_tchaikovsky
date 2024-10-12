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

    }

    /**
     * Method for getting the inputs used by the Shooter Subsystem (From Robot - To Code)
     * @return ShooterIOInputs
     */
    public ShooterIOInputs getInputs();

    /**
     * Method for setting the outputs from the Shooter Subsystem (To Robot - From Code)
     * @param outputs
     */
    public void setOutputs(ShooterIOOutputs outputs);

    /**
     * Method for reseting the LEDs to their defualt state
     */
    public void ResetLEDs();

    /**
     * Method for running the LED pattern that plays when a note is detected
     */
    public void RunNoteDetectedLEDPattern();

    /**
     * Method for running the LED pattern that plays when shooting
     */
    public void RunShootingNoteLEDPattern();
    
    /**
     * Method that stops the shooter motors
     */
    public void StopMotors();

    /**
     * Method that runs the shooter motors at a desired speed
     * @param speed The speed to run the shooter motors at
     */
    public void RunShooter(double speed);

    /**
     * Method to run the green wheel in the shooter
     * @param speed The speed to run the green wheel at
     */
    public void RunGreenWheel(double speed);

    /**
     * Method that sets the value of the elevator solenoid
     * @param value The value the solenoid should be set to
     */
    public void SetElevator(Value value);
}