#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// #define DEBUG_BLINK
// #define DEBUG_PULSE
// #define DEBUG_RACE

enum LEDPattern {
  Solid,
  Blink,
  Race,
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

    LEDSection(byte r, byte g, byte b, LEDPattern pattern, uint16_t speed, bool direction) {
      color = packColor(r, g, b);

      this->pattern = pattern;
      this->speed = speed;
      this->direction = direction;
    }

    LEDSection(byte r, byte g, byte b, LEDPattern pattern, uint16_t speed, bool direction, byte frame) {
      // pack color into a single 32-bit integer
      color = packColor(r, g, b);

      this->pattern = pattern;
      this->speed = speed;
      this->direction = direction;
      this->frame = frame;
    }

    LEDSection() { }

    uint32_t color = 0;
    LEDPattern pattern = Solid;
    uint16_t speed = 0;
    bool direction = 0;

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

#define PIN1 3 // D1
#define NUMPIXELS 78
#define SECTION_COUNT 3
#define LEDS_PER_SECTION 26

Adafruit_NeoPixel strip(NUMPIXELS, PIN1, NEO_GRB + NEO_KHZ800);
LEDSection sectionStateBuffer[SECTION_COUNT] = {
  // Section, R, G, B, Pattern, Speed, Direction
  LEDSection(255, 0, 0, Pulse, 1000 / 25, true),
  LEDSection(0, 255, 0, Blink, 1000, 0),
  LEDSection(0, 0, 255, Pulse, 1000 / 25, false),
};
LEDSection sectionStates[SECTION_COUNT] = {
  // Section, R, G, B, Pattern, Speed, Direction
  LEDSection(),
  LEDSection(),
  LEDSection(),
};

void receiveData(int byteCount) {
    Serial.println("Received " + String(byteCount) + " I2C bytes");

    while (Wire.available() > 7) {
      Serial.println("Received packet");
      LEDSection packet;

      byte sectionNum = Wire.read(); // Section (1 byte)

      // Validate the section number
      if (sectionNum < 0 || sectionNum >= SECTION_COUNT) {
        Serial.println("Received packet for invalid section: " + sectionNum);

        // Skip the rest of the packet
        for (int i = 1; i < 7; i++) {
          Wire.read();
        }
      }
      
      // Read the color (3 bytes)
      byte r = Wire.read(); // R
      byte g = Wire.read(); // G
      byte b = Wire.read(); // B
      packet.color = packet.packColor(r, g, b);

      packet.pattern = (LEDPattern)Wire.read(); // Pattern (1 byte)
      packet.speed = Wire.read() * 10; // Speed (1 bytes)
      packet.direction = Wire.read() > 0; // Direction (1 byte)
      
      // Save the packet to the section buffer
      sectionStateBuffer[sectionNum] = packet;
    }

    // Clear any extra data in the Wire buffer that isn't a full packet
    while (Wire.available() > 0) {
      Wire.read();
    }
}

void setup() {
  // Set up serial comms
  Serial.begin(115200);

  // Setup I2C
  Wire.begin(8);  // Set the device address to 8
//   Wire.begin(10);  // Set the device address to 10
  Wire.onReceive(receiveData);

  // Set up LED strip
  strip.begin();
}

void loop() {
  // Update each section of the LED strip
  for (int i = 0; i < SECTION_COUNT; i++) {
    updateSection(i);
  }
  delay(1);
}

void updateSection(int section) {
  // If the buffer data is different from the section state, set the new section pattern
  if (sectionStateBuffer[section] != sectionStates[section]) {
    if (sectionStateBuffer[section].pattern == Solid) {
        setSolid(section, sectionStateBuffer[section].color);
        return;
    }

    // Write debug info for each pattern
    switch (sectionStateBuffer[section].pattern) {
      case Blink:
        #ifdef DEBUG_BLINK
            Serial.println("Starting Blink pattern for section " + String(section) + " at " + String(data.lastFrameTimestamp) + "ms");
            Serial.println("Next update time: " + String(data.lastFrameTimestamp + data.speed) + "ms -- Current time: " + String(data.lastFrameTimestamp) + "ms");
        #endif
        break;
      case Race:
        #ifdef DEBUG_RACEFORWARD
            Serial.println("Starting Race pattern for section " + String(section) + " at " + String(data.lastFrameTimestamp) + "ms");
            Serial.println("Next update time: " + String(data.lastFrameTimestamp + data.speed) + "ms -- Current time: " + String(data.lastFrameTimestamp) + "ms");
        #endif
        break;
      case Pulse:
        #ifdef DEBUG_PULSE
            Serial.println("Starting Pulse pattern for section " + String(section) + " at " + String(data.lastFrameTimestamp) + "ms");
            Serial.println("Next update time: " + String(data.lastFrameTimestamp + data.speed) + "ms -- Current time: " + String(data.lastFrameTimestamp) + "ms");
        #endif
        break;
    }

    // Move the buffer to the current state
    sectionStates[section] = sectionStateBuffer[section];

    // Set section to frame 0: LEDs off
    // Save the timestamp
    setSolid(section, 0);
    sectionStates[section].frame = 0;
    sectionStates[section].lastFrameTimestamp = millis();
  } else {
    // No change, update the section's pattern
    switch (sectionStates[section].pattern) {
      case Blink:
        updateBlink(section, sectionStateBuffer[section]);
        break;
      case Race:
        updateRace(section, sectionStateBuffer[section]);
        break;
      case Pulse:
        updatePulse(section, sectionStateBuffer[section]);
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

// Blink pattern
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

// Pulse pattern
const byte PULSE_FRAMES = 25;
const byte MAX_BRIGHTNESS = 250;
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

    // If the direction is reversed, invert the brightness
    if (!data.direction) {
      frameBrightness = 1 - frameBrightness;
    }

    // Set the section to the color and brightness
    setSolid(section, data.r() * frameBrightness, data.g() * frameBrightness, data.b() * frameBrightness);

    // Save the timestamp
    data.lastFrameTimestamp = millis();
  }
}

// Race pattern
const int RACE_LED_COUNT = 4;
const int RACE_FRAME_COUNT = LEDS_PER_SECTION + RACE_LED_COUNT;
void updateRace(int section, LEDSection& data) {
  long currentTime = millis();

  // If the current time is > the last frame timestamp + speed, update the section
  if (currentTime - data.lastFrameTimestamp >= data.speed) {
    if (data.direction == 1) {
        // Move the illuminated LEDs forward by 2 pixels per frame
        data.frame += 2;

        // If the frame exceeds the number of LEDs + RACE_LED_COUNT, reset to frame 0
        if (data.frame >= RACE_FRAME_COUNT) {
            data.frame = 0;
        }
    } else {
        // Move the illuminated LEDs backward by 2 pixels per frame
        data.frame -= 2;

        // If the frame exceeds the number of LEDs + RACE_LED_COUNT, reset to frame 0
        if (data.frame <= 0) {
            data.frame = RACE_FRAME_COUNT;
        }
    }

    // Move 4 LEDs forward by 2 pixels per frame using the color specified in the data
    int sectionIndex = section * LEDS_PER_SECTION;
    for (int i = 0; i < LEDS_PER_SECTION; i++) {
        int illuminatedUpperLimit = data.frame;
        int illuminatedLowerLimit = illuminatedUpperLimit - RACE_LED_COUNT;
        
        if (i <= illuminatedUpperLimit && i > illuminatedLowerLimit) {
          strip.setPixelColor(sectionIndex + i, data.color);
        } else {
          strip.setPixelColor(sectionIndex + i, 0);
        }
    }

    strip.show();

    // Save the timestamp
    data.lastFrameTimestamp = currentTime;
  }
}