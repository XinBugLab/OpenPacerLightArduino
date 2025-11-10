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

static inline void applyBrightness(int v) {
  analogWrite(ledPwmPin, clamp255(v));
}

// opcode constants (3-bit opcode in highest bits of id_mask)
const uint8_t OPCODE_LIGHT = 0;   // 0b000: light control
const uint8_t OPCODE_SET_ID = 1;  // 0b001: set device id (single packet)
const int KEY_SET_ID = 165;       // simple key to guard ID setting (0xA5)

// LED fade/hold state machine
enum LedPhase { LED_IDLE, LED_FADE_IN, LED_HOLD, LED_FADE_OUT };
LedPhase ledPhase = LED_IDLE;
uint8_t ledTarget = 0;
uint8_t ledCurrent = 0;
unsigned long msPhaseStart = 0;
unsigned long msFadeIn = 0;
unsigned long msHold = 0;
unsigned long msFadeOut = 0;

void queueLedAction(int target, unsigned long fadeIn, unsigned long hold, unsigned long fadeOut) {
  ledTarget = (uint8_t)clamp255(target);
  msFadeIn = fadeIn;
  msHold = hold;
  msFadeOut = fadeOut;
  ledPhase = LED_FADE_IN;
  msPhaseStart = millis();
}

void updateLed() {
  unsigned long now = millis();
  switch (ledPhase) {
    case LED_IDLE:
      // keep current brightness
      break;
    case LED_FADE_IN: {
      if (msFadeIn == 0) {
        ledCurrent = ledTarget;
        applyBrightness(ledCurrent);
        ledPhase = LED_HOLD;
        msPhaseStart = now;
        break;
      }
      unsigned long elapsed = now - msPhaseStart;
      if (elapsed >= msFadeIn) {
        ledCurrent = ledTarget;
        applyBrightness(ledCurrent);
        ledPhase = LED_HOLD;
        msPhaseStart = now;
        break;
      }
      float progress = (float)elapsed / (float)msFadeIn;
      int value = (int)(progress * ledTarget + 0.5f);
      ledCurrent = (uint8_t)clamp255(value);
      applyBrightness(ledCurrent);
      break;
    }
    case LED_HOLD: {
      // ensure brightness stays at target
      applyBrightness(ledTarget);
      if (msHold == 0 || (now - msPhaseStart) >= msHold) {
        ledPhase = LED_FADE_OUT;
        msPhaseStart = now;
      }
      break;
    }
    case LED_FADE_OUT: {
      if (msFadeOut == 0) {
        ledCurrent = 0;
        applyBrightness(0);
        ledPhase = LED_IDLE;
        break;
      }
      unsigned long elapsed = now - msPhaseStart;
      if (elapsed >= msFadeOut) {
        ledCurrent = 0;
        applyBrightness(0);
        ledPhase = LED_IDLE;
        break;
      }
      float progress = (float)elapsed / (float)msFadeOut;
      int value = (int)((1.0f - progress) * ledTarget + 0.5f);
      ledCurrent = (uint8_t)clamp255(value);
      applyBrightness(ledCurrent);
      break;
    }
  }
}

// Process one complete command line
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
          queueLedAction(b, (unsigned long)cmd.fade_in, (unsigned long)cmd.hold, (unsigned long)cmd.fade_out);
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

// Parse a comma-separated command string according to the given logic
// Returns true on success, false on failure with error message populated
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

      // No WHOAMI command; only handle control commands

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

  // LED
  updateLed();
}