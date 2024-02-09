#include <Adafruit_NeoPixel.h>

// #define DEBUG_BLINK
// #define DEBUG_PULSE

enum LEDPattern {
  Solid,
  Blink,
  RaceForward,
  RaceBackward,
  Pulse,
};

class LEDSection {
  public:
    static uint32_t packColor(byte r, byte g, byte b) {
      uint32_t color = 0;
      color |= ((uint32_t)r << 16);
      color |= ((uint32_t)g << 8);
      color |= b;
      return color;
    }

    LEDSection(byte r, byte g, byte b, LEDPattern pattern, uint16_t speed, byte direction) {
      color = packColor(r, g, b);

      this->pattern = pattern;
      this->speed = speed;
      this->direction = direction;
    }

    LEDSection(byte r, byte g, byte b, LEDPattern pattern, uint16_t speed, byte direction, byte frame) {
      // pack color into a single 32-bit integer
      color |= ((uint32_t)r << 16);
      color |= ((uint32_t)g << 8);
      color |= b;

      this->pattern = pattern;
      this->speed = speed;
      this->direction = direction;
      this->frame = frame;
    }

    LEDSection() { }

    uint32_t color = 0;
    LEDPattern pattern = Solid;
    uint16_t speed = 0;
    byte direction = 0;

    // Pattern state machine
    byte frame = 0;
    long lastFrameTimestamp = 0;

    byte r() {
      return (color >> 16) & 0xFF;
    }

    byte g() {
      return (color >> 8) & 0xFF;
    }

    byte b() {
      return color & 0xFF;
    }

    byte alpha() {
      return color >> 24;
    }

    bool operator==(const LEDSection& other) const {
        return (
            this->color == other.color &&
            this->pattern == other.pattern &&
            this->speed == other.speed &&
            this->direction == other.direction
        );
    }

    bool operator!=(const LEDSection& other) const {
        return !(*this == other);
    }
};

#define PIN 3 // D1
#define NUMPIXELS 6 
#define SECTION_COUNT 3
#define LEDS_PER_SECTION 2

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
LEDSection sectionDataBuffer[SECTION_COUNT] = {
  // Section, R, G, B, Pattern, Speed, Direction
  LEDSection(255, 0, 0, Pulse, 1000 / 25, 1),
  LEDSection(0, 255, 0, Blink, 1000, 0),
  LEDSection(0, 0, 255, Pulse, 1000 / 25, 0),
};
LEDSection SECTION_COUNTtates[SECTION_COUNT] = {
  // Section, R, G, B, Pattern, Speed, Direction
  LEDSection(),
  LEDSection(),
  LEDSection(),
};

void setup() {
  // Set up serial comms
  Serial.begin(115200);

  // Set up LED strip
  strip.begin();
}

void loop() {
  // If we have serial data in the buffer, read 1 or more 8-byte packets, one packet per section.
  if (Serial.available() > 7) {
    while (Serial.available() > 7) {
      LEDSection packet;

      byte sectionNum = Serial.read(); // Section (1 byte)

      // Validate the section number
      if (sectionNum < 0 || sectionNum >= SECTION_COUNT) {
        Serial.println("Received packet for invalid section " + sectionNum);

        // Skip the rest of the packet
        for (int i = 1; i < 8; i++) {
          Serial.read();
        }
      }
      
      // Read the color (3 bytes)
      byte r = Serial.read(); // R
      byte g = Serial.read(); // G
      byte b = Serial.read(); // B
      packet.color = packet.packColor(r, g, b);

      // Pattern (1 byte)
      packet.pattern = (LEDPattern)Serial.read(); 

      // Speed (2 bytes)
      byte speed1 = Serial.read();
      byte speed2 = Serial.read();
      packet.speed = (static_cast<uint16_t>(speed1) << 8) | speed2;

      // Direction (1 byte)
      packet.direction = Serial.read();
      
      sectionDataBuffer[sectionNum] = packet;
    }

    // Clear any extra data lying in the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  // Update each section of the LED strip
  for (int i = 0; i < SECTION_COUNT; i++) {
    updateSection(i);
  }
  delay(1);
}

void updateSection(int section) {
  // If the new data is different from the current state, update the LED strip
  if (sectionDataBuffer[section] != SECTION_COUNTtates[section]) {
    // Update the LED strip
    switch (sectionDataBuffer[section].pattern) {
      case Solid:
        setSolid(section, sectionDataBuffer[section].color);
        break;
      case Blink:
        setBlink(section, sectionDataBuffer[section]);
        break;
      case RaceForward:
        // setRaceForward(section, sectionDataBuffer[section]);
        break;
      case RaceBackward:
        // setRaceBackward(section, sectionDataBuffer[section]);
        break;
      case Pulse:
        setPulse(section, sectionDataBuffer[section]);
        break;
    }

    // Update the current state in memory from the buffer
    SECTION_COUNTtates[section] = sectionDataBuffer[section];
  } else {
    // No change, update the section's pattern
    switch (SECTION_COUNTtates[section].pattern) {
      case Blink:
        updateBlink(section, sectionDataBuffer[section]);
        break;
      case RaceForward:
        // updateRaceForward(section, sectionDataBuffer[section]);
        break;
      case RaceBackward:
        // updateRaceBackward(section, sectionDataBuffer[section]);
        break;
      case Pulse:
        updatePulse(section, sectionDataBuffer[section]);
        break;
    }
  }
}

void setSolid(int section, uint32_t color) {
  // Set the color of each pixel in the section
  int sectionIndex = section * LEDS_PER_SECTION;
  for (int i = 0; i < LEDS_PER_SECTION; i++) {
    strip.setPixelColor(sectionIndex + i, color);
  }

  // Show the changes on the LED strip
  strip.show();
}

void setSolid(int section, byte r, byte g, byte b) {
  // Set the color of each pixel in the section
  int sectionIndex = section * LEDS_PER_SECTION;
  for (int i = 0; i < LEDS_PER_SECTION; i++) {
    strip.setPixelColor(sectionIndex + i, r, g, b);
  }

  // Show the changes on the LED strip
  strip.show();
}

#pragma region Blink Pattern
void setBlink(int section, LEDSection& data) {
  // Set blink pattern
  // Set section to frame 0: LEDs off
  // Save the timestamp
  setSolid(section, 0);
  data.frame = 0;
  data.lastFrameTimestamp = millis();
  #ifdef DEBUG_BLINK
    Serial.println("Starting blink pattern for section " + String(section) + " at " + String(data.lastFrameTimestamp) + "ms");
    Serial.println("Next update time: " + String(data.lastFrameTimestamp + data.speed) + "ms -- Current time: " + String(data.lastFrameTimestamp) + "ms");
  #endif
}

void updateBlink(int section, LEDSection& data) {
  long currentTime = millis();

  // If the current time is > the last frame timestamp + speed, update the section
  if (currentTime - data.lastFrameTimestamp >= data.speed) {
    #ifdef DEBUG_BLINK
      Serial.println("Updating blink pattern for section " + String(section));
      Serial.println("Next update time: " + String(data.lastFrameTimestamp + data.speed) + "ms -- Current time: " + String(currentTime) + "ms");
    #endif
    // Toggle the frame
    data.frame = (data.frame == 0) ? 1 : 0;

    if (data.frame == 0) {
      // Set section to frame 1: LEDs off
      setSolid(section, data.color);
    } else {
      // Set section to frame 0: LEDs off
      setSolid(section, 0);
    }

    // Save the timestamp
    data.lastFrameTimestamp = currentTime;
  }
}
#pragma endregion

#pragma region Pulse Pattern
const byte PULSE_FRAMES = 25;
const byte MAX_BRIGHTNESS = 250;
void setPulse(int section, LEDSection& data) {
  // Set pulse pattern
  // Set section to frame 0: LEDs off
  // Save the timestamp
  setSolid(section, 0);
  data.frame = 0;
  data.lastFrameTimestamp = millis();
  #ifdef DEBUG_PULSE
    Serial.println("Starting pulse pattern for section " + String(section) + " at " + String(data.lastFrameTimestamp) + "ms");
    Serial.println("Next update time: " + String(data.lastFrameTimestamp + data.speed) + "ms -- Current time: " + String(data.lastFrameTimestamp) + "ms");
  #endif
}

void updatePulse(int section, LEDSection& data) {
  long currentTime = millis();

  // If the current time is > the last frame timestamp + speed, update the section
  if (currentTime - data.lastFrameTimestamp >= data.speed) {
    // If the frame is 0, set the section to the color and brightness
    if (data.frame == 25) {
      // Reset to frame 0
      data.frame = 0;
    } else {
      data.frame++;
    }

    // Calculate brightness multiplier (0-25 frames, 0-1 brightness)
    float frameBrightness = (float)data.frame / (float)PULSE_FRAMES;

    if (data.direction == 1) {
      frameBrightness = 1 - frameBrightness;
    }

    // Set the section to the color and brightness
    setSolid(section, data.r() * frameBrightness, data.g() * frameBrightness, data.b() * frameBrightness);

    // Save the timestamp
    data.lastFrameTimestamp = millis();
  }
}
#pragma endregion