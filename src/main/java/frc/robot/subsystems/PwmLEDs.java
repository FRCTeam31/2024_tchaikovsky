package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
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

  public PwmLEDs(LEDConfig config) {
    m_config = config;
    // Initialize the LED pattern buffers
    m_temporaryPatterns = new LEDPattern[m_config.SectionCount];
    m_persistentPatterns = new LEDPattern[m_config.SectionCount];

    // Initialize the LED strip
    m_led = new AddressableLED(5);
    m_ledBuffer = new AddressableLEDBuffer(78);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the strip to a default pattern
    setStripPersistentPattern(new PulsePattern(Color.WHITE, 4));
    m_led.start();

    // Start the update loop
    updateLoopExecutor.scheduleAtFixedRate(this::ledUpdateLoop, 0, 2, java.util.concurrent.TimeUnit.MILLISECONDS);
  }

  /**
   * Set the pattern of a section of the LED strip and save it as the last pattern
   * @param section The section to set
   * @param pattern The pattern to set the section to
   */
  private void setSectionPersistentPattern(int section, LEDPattern pattern) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_led != null && m_ledBuffer != null) {
      // If the last pattern isTheSameAs the new pattern, don't set it
      if (m_persistentPatterns[section] != null && m_persistentPatterns[section].isSameAs(pattern)) {
        return;
      }

      // Set the pattern and save it
      m_persistentPatterns[section] = pattern;
    }
  }

  /**
   * Set the patterns of all sections of the LED strip
   * @param section The section to set
   * @param pattern The pattern to set the section to
   */
  public void setStripPersistentPattern(LEDPattern pattern) {
    for (int i = 0; i < m_config.SectionCount; i++) {
      setSectionPersistentPattern(i, pattern);
    }
  }

  /**
   * Set the pattern of a section of the LED strip without saving it as the last pattern
   * @param section The section to set
   * @param pattern The pattern to set the section to temporarily
   */
  private void setSectionTemporaryPattern(int section, LEDPattern pattern) {
    if (section < 0 || section >= m_config.SectionCount) {
      System.out.println("[LEDs:WARNING] Invalid LED section: " + section);
    }

    if (m_led != null && m_ledBuffer != null) {
      m_temporaryPatterns[section] = pattern;
    }
  }

  /**
   * Set the patterns of all sections of the LED strip
   * @param section The section to set
   * @param pattern The pattern to set the section to
   */
  public void setStripTemporaryPattern(LEDPattern pattern) {
    if (m_led != null && m_ledBuffer != null) {
      for (int i = 0; i < m_config.SectionCount; i++) {
        setSectionTemporaryPattern(i, pattern);
      }
    }
  }

  /**
   * Set a section of the LED strip back to it's previous pattern
   * @param section The section to set
   * @param pattern The pattern to set the section to
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
   * Set a section of the LED strip back to it's previous pattern
   * @param section The section to set
   * @param pattern The pattern to set the section to
   */
  public void restorePersistentStripState() {
    for (int i = 0; i < m_config.SectionCount; i++) {
      restorePersistentSectionPattern(i);
    }
  }

  public void ledUpdateLoop() {
    try {
      for (int sectionIndex = 0; sectionIndex < m_config.SectionCount; sectionIndex++) {
        // If the temporary pattern for this section is not null, use it instead of the persistent pattern
        var pattern = m_temporaryPatterns[sectionIndex] != null
          ? m_temporaryPatterns[sectionIndex]
          : m_persistentPatterns[sectionIndex];

        // If the pattern is null, skip this section
        if (pattern != null) {
          // Request for the pattern to calculate the next frame and update the buffer
          pattern.updateBuffer(sectionIndex * m_config.PixelsPerSection, m_config.PixelsPerSection, m_ledBuffer);
        }
      }

      // Update the LED strip with the new buffer
      m_led.setData(m_ledBuffer);
    } catch (Exception e) {
      loopErrorCounter++;
      DriverStation.reportError("[LEDs:ERROR] Error in update loop: " + e.getMessage(), e.getStackTrace());

      if (loopErrorCounter > m_config.SectionCount) {
        var msg = "[LEDs:ERROR] LED update loop has failed " + m_config.SectionCount + " times. Stopping loop.";
        DriverStation.reportError(msg, false);
        System.out.println(msg);
        updateLoopExecutor.shutdown();
      }
    }
  }
}
