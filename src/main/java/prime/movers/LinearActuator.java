package prime.movers;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Relay;

public class LinearActuator {

  private Relay relay;
  private AnalogInput potentiometer;
  private final double MAX_VOLTAGE = 5.0;

  public LinearActuator(int relayChannel, int potentiometerAnalogChannel) {
    relay = new Relay(relayChannel, Relay.Direction.kBoth);
    potentiometer = new AnalogInput(potentiometerAnalogChannel);
  }

  public void runForward() {
    relay.setDirection(Relay.Direction.kForward);
    relay.set(Relay.Value.kOn);
  }

  public void runReverse() {
    relay.setDirection(Relay.Direction.kReverse);
    relay.set(Relay.Value.kOn);
  }

  public void stop() {
    relay.set(Relay.Value.kOff);
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
