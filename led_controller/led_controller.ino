#include <Adafruit_NeoPixel.h>
#include <Thread.h>
#include <ThreadController.h>

enum LEDPattern {
  Solid,
  Blink,
  RaceForward,
  RaceBackward,
  Pulse,
};

class LEDSection {
  public:
    LEDSection(byte r, byte g, byte b, byte brightness, LEDPattern pattern, byte speed, byte direction, byte frame) {
      this->r = r;
      this->g = g;
      this->b = b;
      this->brightness = brightness;
      this->pattern = pattern;
      this->speed = speed;
      this->direction = direction;
      this->frame = frame;
    }

    LEDSection() { }

    byte r = 0;
    byte g = 0;
    byte b = 0;
    byte brightness = 0;
    LEDPattern pattern = Solid;
    byte speed = 0;
    byte direction = 0;
    byte frame = 0;
    long lastFrameTimestamp = 0;
};

#define PIN 1 // D1
#define NUMPIXELS 6 
#define SECTIONS 3
#define LEDS_PER_SECTION 2

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
LEDSection sectionDataBuffer[SECTIONS] = {
  // Section, R, G, B, Brightness, Pattern, Speed, Direction
  LEDSection(),
  LEDSection(),
  LEDSection(),
};
LEDSection sectionStates[SECTIONS] = {
  // Section, R, G, B, Brightness, Pattern, Speed, Direction
  LEDSection(),
  LEDSection(),
  LEDSection(),
};

#define THREADSPEED 10
ThreadController threadController = ThreadController();

void setup() {
  // Set up serial comms
  Serial.begin(115200);

  // Set up LED strip
  strip.begin();

  // Set up threads
  for (int i = 0; i < SECTIONS; i++) {
    Thread thread = Thread();
    thread.enabled = true;
    thread.setInterval(THREADSPEED);
    
    switch (i) {
      case 0:
        thread.onRun(section1Runner);
        break;
      case 1:
        thread.onRun(section2Runner);
        break;
      case 2:
        thread.onRun(section3Runner);
        break;
    }

    threadController.add(&thread);
  }
}

void loop() {
  // If we have serial data in the buffer, read 1 or more 8-byte packets, one packet per section.
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      byte packet[8] = {};
      int bytesRead = Serial.readBytes(packet, 8);

      if (bytesRead != 8) {
        Serial.println("Received invalid packet of length " + bytesRead);
        continue;
      }

      if (packet[0] < 0 || packet[0] >= SECTIONS) {
        Serial.println("Received packet for invalid section " + packet[0]);
        continue;
      }

      Serial.println("Received RGBBPS packet for section " + String(packet[0]));
      sectionDataBuffer[packet[0]].r = packet[1];
      sectionDataBuffer[packet[0]].g = packet[2];
      sectionDataBuffer[packet[0]].b = packet[3];
      sectionDataBuffer[packet[0]].brightness = packet[4];
      sectionDataBuffer[packet[0]].pattern = (LEDPattern)packet[5];
      sectionDataBuffer[packet[0]].speed = packet[6];
      sectionDataBuffer[packet[0]].direction = packet[7];
    }
  }

  // Update all threads
  threadController.run();
}

void section1Runner() {
  runSection(0);
}

void section2Runner() {
  runSection(1);
}

void section3Runner() {
  runSection(2);
}

void runSection(int section) {
  // If the new data is different from the current state, update the LED strip
  if (sectionDataBuffer[section].r != sectionStates[section].r || 
      sectionDataBuffer[section].g != sectionStates[section].g || 
      sectionDataBuffer[section].b != sectionStates[section].b || 
      sectionDataBuffer[section].brightness != sectionStates[section].brightness || 
      sectionDataBuffer[section].pattern != sectionStates[section].pattern || 
      sectionDataBuffer[section].speed != sectionStates[section].speed) {
    // Update the LED strip
    switch (sectionDataBuffer[section].pattern) {
      case Solid:
        setSolid(section, sectionDataBuffer[section].r, sectionDataBuffer[section].g, sectionDataBuffer[section].b, sectionDataBuffer[section].brightness);
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
        // setPulse(section, sectionDataBuffer[section]);
        break;
    }

    // Update the current state of the sections in memory
    sectionStates[section] = sectionDataBuffer[section];
  } else {
    // No change, update the section's pattern
    switch (sectionStates[section].pattern) {
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
        // updatePulse(section, sectionDataBuffer[section]);
        break;
    }
  }
}

void setSolid(int section, byte r, byte g, byte b, byte brightness) {
  // Set the color and brightness of each pixel in the section
  int sectionIndex = section * LEDS_PER_SECTION;
  for (int i = 0; i < LEDS_PER_SECTION; i++) {
    strip.setPixelColor(sectionIndex + i, strip.Color(r, g, b));
  }

  // Set the brightness of the section
  strip.setBrightness(brightness);

  // Show the changes on the LED strip
  strip.show();
}

#pragma region Blink Pattern
void setBlink(int section, LEDSection data) {
  // Set blink pattern
  // Set section to frame 0: LEDs off
  // Save the timestamp
  setSolid(section, data.r, data.g, data.b, 0);
  data.frame = 0;
  data.lastFrameTimestamp = millis();
}

void updateBlink(int section, LEDSection data) {
  // If the current time is > the last frame timestamp + speed, update the section
  if (millis() > data.lastFrameTimestamp + data.speed) {
    // If the frame is 0, set the section to the color and brightness
    if (data.frame == 0) {
      setSolid(section, data.r, data.g, data.b, data.brightness);
      data.frame = 1;
    } else {
      // If the frame is 1, set the section to off
      setSolid(section, data.r, data.g, data.b, 0);
      data.frame = 0;
    }

    // Save the timestamp
    data.lastFrameTimestamp = millis();
  }
}
#pragma endregion

#pragma region Pulse Pattern
const byte PULSE_FRAMES = 25;
const byte MAX_BRIGHTNESS = 250;
void setPulse(int section, LEDSection data) {
  // Set pulse pattern
  // Set section to frame 0: LEDs off
  // Save the timestamp
  setSolid(section, data.r, data.g, data.b, 0);
  data.frame = 0;
  data.lastFrameTimestamp = millis();
}

void updatePulse(int section, LEDSection data) {
  // If the current time is > the last frame timestamp + speed, update the section
  if (millis() > data.lastFrameTimestamp + data.speed) {
    // If the frame is 0, set the section to the color and brightness
    if (data.frame == 25) {
      // Reset to frame 0
      setSolid(section, data.r, data.g, data.b, 0);
      data.frame = 0;
      data.lastFrameTimestamp = millis();
    } else {
      // Calculate the new brightness, one step up from the previous frame
      byte frameBrightness = (MAX_BRIGHTNESS / PULSE_FRAMES) * data.frame;
      // Set the section to the color and brightness
      setSolid(section, data.r, data.g, data.b, frameBrightness);
      data.frame++;
    }

    // Save the timestamp
    data.lastFrameTimestamp = millis();
  }
}
#pragma endregion