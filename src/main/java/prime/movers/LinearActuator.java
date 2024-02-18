package prime.movers;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;

public class LinearActuator {

  private AnalogInput potentiometer;
  private final double MAX_VOLTAGE = 5.0;
  VictorSPX m_linearActuatorVictorSPX;

  public LinearActuator(int victorCanID, int potentiometerAnalogChannel) {
    potentiometer = new AnalogInput(potentiometerAnalogChannel);
    m_linearActuatorVictorSPX = new VictorSPX(victorCanID);
  }

  public void runForward() {
    m_linearActuatorVictorSPX.set(VictorSPXControlMode.PercentOutput, 1);
  }

  public void runReverse() {
    m_linearActuatorVictorSPX.set(VictorSPXControlMode.PercentOutput, -1);
  }

  public void stop() {
    m_linearActuatorVictorSPX.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public double getPosition() {
    return potentiometer.getVoltage() / MAX_VOLTAGE;
  }

  public void set(double positionPercent) {
    if (positionPercent > getPosition()) {
      while (getPosition() < positionPercent) {
        runForward();
      }
    } else if (positionPercent < getPosition()) {
      while (getPosition() > positionPercent) {
        runReverse();
      }
    }

    stop();
  }
}
