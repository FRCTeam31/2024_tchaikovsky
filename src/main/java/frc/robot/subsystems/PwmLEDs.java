package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.LEDConfig;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import prime.control.LEDs.Color;
import prime.control.LEDs.Patterns.LEDPattern;
import prime.control.LEDs.Patterns.PulsePattern;

public class PwmLEDs extends SubsystemBase {

  private LEDConfig m_config;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private final ScheduledExecutorService updateLoopExecutor = Executors.newScheduledThreadPool(1);
  private byte loopErrorCounter = 0;
  private LEDPattern m_persistentPatterns[];
  private LEDPattern m_temporaryPatterns[];

  public PwmLEDs(LEDConfig m_config) {
    m_led = new AddressableLED(5);
    m_ledBuffer = new AddressableLEDBuffer(78);

    // Set the LEDs to a default state
    setStripPersistentPattern(new PulsePattern(Color.WHITE, 4));
    m_temporaryPatterns = new LEDPattern[m_config.SectionCount];
    m_led.start();
    updateLoopExecutor.scheduleAtFixedRate(this::ledUpdateLoop, 0, 2, java.util.concurrent.TimeUnit.MILLISECONDS);
  }

  /**
   * Set the state of a section of the LED strip and save it as the last state
   * @param section The section to set
   * @param state The state to set the section to
   */
  private void setSectionPersistentPattern(int section, LEDPattern state) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_led != null && m_ledBuffer != null) {
      // If the last state isTheSameAs the new state, don't set it
      if (m_persistentPatterns[section] != null && m_persistentPatterns[section].isSameAs(state)) {
        return;
      }

      // Set the state and save it
      m_persistentPatterns[section] = state;
    }
  }

  /**
   * Set the state of a section of the LED strip without saving it as the last state
   * @param section The section to set
   * @param state The state to set the section to temporarily
   */
  private void setSectionTemporaryPattern(int section, LEDPattern state) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_led != null && m_ledBuffer != null) {
      // TODO
    }
  }

  /**
   * Set the states of all sections of the LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setStripPersistentPattern(LEDPattern state) {
    if (m_led != null && m_ledBuffer != null) {
      for (int i = 0; i < m_config.SectionCount; i++) {
        setSectionPersistentPattern(i, state);
      }
    }
  }

  /**
   * Set the states of all sections of the LED strip
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void setStripTemporaryPattern(LEDPattern state) {
    if (m_led != null && m_ledBuffer != null) {
      for (int i = 0; i < m_config.SectionCount; i++) {
        setSectionTemporaryPattern(i, state);
      }
    }
  }

  /**
   * Set a section of the LED strip back to it's previous state
   * @param section The section to set
   * @param state The state to set the section to
   */
  private void restorePersistentSectionPattern(int section) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_temporaryPatterns[section] != null) {
      m_temporaryPatterns[section] = null;
    }
  }

  /**
   * Set a section of the LED strip back to it's previous state
   * @param section The section to set
   * @param state The state to set the section to
   */
  public void restorePersistentStripState() {
    for (int i = 0; i < m_config.SectionCount; i++) {
      restorePersistentSectionPattern(i);
    }
  }

  public void ledUpdateLoop() {
    for (int i = 0; i < m_config.SectionCount; i++) {
      try { // Always try to use the persistent pattern first
        // If the temporary pattern for this section is not null, use it instead of the persistent pattern
        var pattern = m_persistentPatterns[i];
        if (m_temporaryPatterns[i] != null) {
          pattern = m_temporaryPatterns[i];
        }

        if (pattern == null) {
          continue;
        }

        // Request for the buffer to be updated by the pattern
        pattern.updateBuffer(i * m_config.PixelsPerSection, m_config.PixelsPerSection, m_ledBuffer);
        m_led.setData(m_ledBuffer);
      } catch (Exception e) {
        loopErrorCounter++;
        if (loopErrorCounter > m_config.SectionCount) {
          System.out.println(
            "[LEDs:ERROR] LED update loop has failed " + m_config.SectionCount + " times. Stopping loop."
          );
          updateLoopExecutor.shutdown();
        }
      }
    }
  }
}
