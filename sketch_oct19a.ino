#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <stdlib.h>

const int swRX = 2;
const int swTX = 4;
SoftwareSerial swSerial(swRX, swTX);

// PWM led control
const int ledPwmPin = 6;
const int wlModePin = 7;
const int addrDeviceID = 2;

union DATA {
  short DeviceID;
  byte hex[2];
};

// Command parsed from swSerial input: id_mask(hex), brightness, fade_in, hold, fade_out
struct Command {
  unsigned long id_mask;
  int brightness;
  int fade_in;
  int hold;
  int fade_out;
};

// Cached device ID read at setup
uint16_t gDeviceID = 0;

// swSerial line buffer for non-blocking reads
char swLineBuf[128];
int swLineLen = 0;

// small helpers
static inline int clamp255(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

static inline bool shouldRespond(unsigned long mask) {
  return (mask & gDeviceID) == gDeviceID;
}

// Encapsulated LED controller to avoid global state
class LedController {
public:
  /**
   * @brief Construct a new Led Controller object.
   * @param pin The PWM pin for the LED.
   */
  explicit LedController(int pin)
    : pwmPin(pin), phase(LED_IDLE), target(0), current(0),
      phaseStart(0), fadeIn(0), hold(0), fadeOut(0) {}

  /**
   * @brief Queues a new LED action (fade-in, hold, fade-out).
   * @param targetVal Target brightness (0-255).
   * @param fIn Fade-in duration in milliseconds.
   * @param h Hold duration in milliseconds.
   * @param fOut Fade-out duration in milliseconds.
   */
  void queueAction(int targetVal, unsigned long fIn, unsigned long h, unsigned long fOut) {
    target = (uint8_t)clamp255(targetVal);
    fadeIn = fIn;
    hold = h;
    fadeOut = fOut;
    phase = LED_FADE_IN;
    phaseStart = millis();
  }

  /**
   * @brief Updates the LED state machine. Should be called in the main loop.
   */
  void update() {
    unsigned long now = millis();
    switch (phase) {
      case LED_IDLE:
        // keep current brightness
        break;
      case LED_FADE_IN: {
        if (phaseComplete(now, fadeIn)) {
          transitionTo(LED_HOLD, target, now);
          break;
        }
        float progress = progressOf(now, fadeIn);
        int value = lerpRounded(0, target, progress);
        current = (uint8_t)clamp255(value);
        applyBrightnessLocal(current);
        break;
      }
      case LED_HOLD: {
        // ensure brightness stays at target
        applyBrightnessLocal(target);
        if (phaseComplete(now, hold)) {
          beginPhase(LED_FADE_OUT, now);
        }
        break;
      }
      case LED_FADE_OUT: {
        if (phaseComplete(now, fadeOut)) {
          transitionTo(LED_IDLE, 0, now);
          break;
        }
        float progress = progressOf(now, fadeOut);
        int value = lerpRounded(target, 0, progress);
        current = (uint8_t)clamp255(value);
        applyBrightnessLocal(current);
        break;
      }
    }
  }

private:
  enum LedPhase { LED_IDLE, LED_FADE_IN, LED_HOLD, LED_FADE_OUT };

  int pwmPin;
  LedPhase phase;
  uint8_t target;
  uint8_t current;
  unsigned long phaseStart;
  unsigned long fadeIn;
  unsigned long hold;
  unsigned long fadeOut;

  /**
   * @brief Linearly interpolates between two values and rounds to the nearest integer.
   * @param start The start value.
   * @param end The end value.
   * @param t The interpolation progress (0.0 to 1.0).
   * @return The interpolated and rounded value.
   */
  static inline int lerpRounded(int start, int end, float t) {
    float v = start + t * (end - start);
    return (int)(v + 0.5f);
  }

  /**
   * @brief Applies a brightness value to the LED.
   * @param v The brightness value (0-255).
   */
  inline void applyBrightnessLocal(int v) {
    analogWrite(pwmPin, clamp255(v));
  }

  /**
   * @brief Checks if the current phase is complete.
   * @param now The current time from millis().
   * @param duration The duration of the phase.
   * @return True if the phase is complete, false otherwise.
   */
  inline bool phaseComplete(unsigned long now, unsigned long duration) {
    return (duration == 0) || ((now - phaseStart) >= duration);
  }

  /**
   * @brief Begins a new phase.
   * @param next The next phase to transition to.
   * @param now The current time from millis().
   */
  inline void beginPhase(LedPhase next, unsigned long now) {
    phase = next;
    phaseStart = now;
  }

  /**
   * @brief Transitions to a new phase with a specific brightness.
   * @param next The next phase.
   * @param brightness The brightness to set.
   * @param now The current time from millis().
   */
  inline void transitionTo(LedPhase next, uint8_t brightness, unsigned long now) {
    current = brightness;
    applyBrightnessLocal(brightness);
    beginPhase(next, now);
  }

  /**
   * @brief Calculates the progress of the current phase.
   * @param now The current time from millis().
   * @param duration The total duration of the phase.
   * @return The progress as a float from 0.0 to 1.0.
   */
  inline float progressOf(unsigned long now, unsigned long duration) {
    if (duration == 0) return 1.0f;
    unsigned long elapsed = now - phaseStart;
    float t = (float)elapsed / (float)duration;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return t;
  }
};

// Single controller instance
static LedController gLed(ledPwmPin);

// opcode constants (3-bit opcode in highest bits of id_mask)
const uint8_t OPCODE_LIGHT = 0;   // 0b000: light control
const uint8_t OPCODE_SET_ID = 1;  // 0b001: set device id (single packet)
const int KEY_SET_ID = 165;       // simple key to guard ID setting (0xA5)

/**
 * @brief Process one complete command line.
 * @param line The command line string to process.
 */
void processCommandLine(const String &line) {
  Command cmd; String err;
  if (parseCommandString(line, cmd, err)) {
    // Extract 3-bit opcode (highest bits) and 13-bit target
    uint8_t opcode = (uint16_t)((cmd.id_mask >> 13) & 0x7);
    uint16_t target = (uint16_t)(cmd.id_mask & 0x1FFF);
    uint16_t my = (uint16_t)(gDeviceID & 0x1FFF);

    switch (opcode) {
      case OPCODE_LIGHT: {
        // Match by mask or exact; allow broadcast (0x1FFF)
        bool match = (target == my) || ((target & my) == my) || (target == 0x1FFF);
        if (match) {
          int b = clamp255(cmd.brightness);
          gLed.queueAction(b, (unsigned long)cmd.fade_in, (unsigned long)cmd.hold, (unsigned long)cmd.fade_out);
        }
        break;
      }
      case OPCODE_SET_ID: {
        // Only exact match and key allowed
        if (target == my && cmd.fade_out == KEY_SET_ID) {
          uint16_t new_id = (uint16_t)(((uint16_t)cmd.fade_in << 8) | ((uint16_t)(cmd.brightness & 0xFF)));
          new_id &= 0x1FFF; // constrain to 13 bits
          if (new_id != 0x0000 && new_id != 0x1FFF) {
            // write EEPROM low then high, mirror reading logic
            EEPROM.write(addrDeviceID, (byte)(new_id & 0xFF));
            EEPROM.write(addrDeviceID + 1, (byte)((new_id >> 8) & 0xFF));
            gDeviceID = new_id;
          }
        }
        break;
      }
      default:
        // Unsupported opcode: ignore silently
        break;
    }
  } else {
    Serial.println("ERR");
  }
}

/**
 * @brief Parse a comma-separated command string.
 * @param command_string The string to parse.
 * @param out The Command struct to populate.
 * @param error An error message if parsing fails.
 * @return True on success, false on failure.
 */
bool parseCommandString(const String &command_string, Command &out, String &error) {
  // Split by comma into exactly 5 parts
  String parts[5];
  int partIndex = 0;
  int start = 0;
  int len = command_string.length();
  for (int i = 0; i < len && partIndex < 5; i++) {
    if (command_string[i] == ',') {
      parts[partIndex++] = command_string.substring(start, i);
      start = i + 1;
    }
  }
  // Last token
  if (partIndex < 5) {
    parts[partIndex++] = command_string.substring(start);
  }

  // Trim spaces
  for (int i = 0; i < partIndex; i++) {
    parts[i].trim();
  }

  if (partIndex != 5) {
    error = "Invalid command format: must have 5 parts";
    return false;
  }

  // parts[0] is hex id_mask
  char *endptr = NULL;
  const char *hexStr = parts[0].c_str();
  unsigned long mask = strtoul(hexStr, &endptr, 16);
  if (endptr == hexStr || (endptr && *endptr != '\0')) {
    error = "Invalid id_mask hex";
    return false;
  }

  out.id_mask = mask;
  out.brightness = parts[1].toInt();
  out.fade_in = parts[2].toInt();
  out.hold = parts[3].toInt();
  out.fade_out = parts[4].toInt();

  error = String();
  return true;
}

/**
 * @brief Initializes the hardware and serial communication.
 */
void setup() {
  pinMode(ledPwmPin, OUTPUT);
  pinMode(wlModePin, OUTPUT);
  
  // Wireless Init
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  swSerial.begin(9600);
  while (!swSerial) {
    ;
  }

  // Keep module in transparent mode by default
  digitalWrite(wlModePin, HIGH);

  // Read device ID once from EEPROM and cache
  {
    DATA id = {0};
    id.hex[0] = EEPROM.read(addrDeviceID);
    id.hex[1] = EEPROM.read(addrDeviceID + 1);
    gDeviceID = static_cast<uint16_t>(id.DeviceID);
  }

  // Minimal startup: avoid verbose serial logs
}

/**
 * @brief Main loop: handles serial communication and updates the LED.
 */
void loop() {
  if(Serial.available()) {
    swSerial.write(Serial.read());
  }

  // Non-blocking read: accumulate characters until newline, then parse
  while (swSerial.available()) {
    char c = (char)swSerial.read();
    if (c == '\r') continue; // ignore CR
    if (c == '\n') {
      swLineBuf[swLineLen] = '\0';
      String line = String(swLineBuf);
      swLineLen = 0;
      line.trim();
      if (line.length() == 0) {
        continue;
      }

      // 处理命令行
      processCommandLine(line);
    } else {
      if (swLineLen < (int)sizeof(swLineBuf) - 1) {
        swLineBuf[swLineLen++] = c;
      } else {
        swLineLen = 0; // reset on overflow
        Serial.println("ERR");
      }
    }
  }

  // Update LED state
  gLed.update();
}