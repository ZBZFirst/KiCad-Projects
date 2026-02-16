#include <Arduino.h>
#include <math.h>

#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <KeyboardDevice.h>
#include <MouseDevice.h>

// ============================
// BLE device identity (edit)
// ============================
#define BLE_DEVICE_NAME   "PaulsESP32"
#define BLE_MANUFACTURER  "PaulLab"

// ============================
// Pull configuration (flip ONE value)
// ============================
#define USE_PULLUP 1   // 1 = INPUT_PULLUP, 0 = INPUT_PULLDOWN

#if USE_PULLUP
  const uint8_t BUTTON_PULL_MODE = INPUT_PULLUP;
  const uint8_t PRESSED_LEVEL    = LOW;
#else
  const uint8_t BUTTON_PULL_MODE = INPUT_PULLDOWN;
  const uint8_t PRESSED_LEVEL    = HIGH;
#endif

// ============================
// 7-button generic index mapping
// ============================
static const int BTN_COUNT = 7;

uint8_t btnPin[BTN_COUNT] = {
  32, // Button 1
  33, // Button 2
  25, // Button 3
  26, // Button 4
  27, // Button 5
  14, // Button 6 (Group switch)
  12  // Button 7 (Disconnect / reboot)
};

// ============================
// Onboard LED (board-specific)
// ============================
#ifndef LED_BUILTIN
  #define LED_BUILTIN 2  // common on many ESP32 dev boards
#endif

static const bool LED_ACTIVE_LOW = false; // set true if your LED behaves inverted

static inline void setLed(bool on) {
  bool levelHigh = (LED_ACTIVE_LOW ? !on : on);
  digitalWrite(LED_BUILTIN, levelHigh ? HIGH : LOW);
}

// ============================
// Timing knobs
// ============================
static const uint32_t DEBOUNCE_MS   = 35;
static const uint32_t MOUSE_RUN_MS  = 8000;   // one run duration
static const uint32_t MOUSE_TICK_MS = 16;     // ~60Hz

// LED pattern requirements
static const uint32_t LED_BLINK_MS = 100;   // 100ms ON, 100ms OFF
static const uint32_t LED_PAUSE_MS = 2000;  // 2s between sequences

// ============================
// HID devices
// ============================
KeyboardDevice* keyboard;
MouseDevice* mouse;
BleCompositeHID compositeHID(BLE_DEVICE_NAME, BLE_MANUFACTURER, 100);

// IMPORTANT: match the working example (advertise as keyboard)
BLEHostConfiguration bleHostConfig;

// ============================
// Button debounce state
// ============================
uint8_t  btnStable[BTN_COUNT]       = {0};
uint8_t  btnLastRead[BTN_COUNT]     = {0};
uint32_t btnLastChangeMs[BTN_COUNT] = {0};

// ============================
// Mouse run state (non-blocking)
// ============================
bool     mouseRunActive   = false;
uint32_t mouseRunStartMs  = 0;
uint32_t mouseLastTickMs  = 0;

// ============================
// Group / mode state
// ============================
static const uint8_t GROUP_DEFAULT = 1;
static const uint8_t GROUP_WORK    = 2;
static const uint8_t GROUP_AUTO    = 3; // was PHRASES

uint8_t actionGroup = GROUP_DEFAULT;

// ============================
// Group 3: Auto-click / auto-key state
// ============================
static const uint8_t AUTO_OFF    = 0;
static const uint8_t AUTO_LCLICK = 1;
static const uint8_t AUTO_RCLICK = 2;
static const uint8_t AUTO_MCLICK = 3;
static const uint8_t AUTO_KEY_E  = 4;
static const uint8_t AUTO_KEY_F  = 5;

uint8_t  autoMode       = AUTO_OFF;
uint32_t autoLastTickMs = 0;

// speed of the auto click / key spam
static const uint32_t AUTO_TICK_MS = 50; // 50ms = 20 actions/sec

// ============================
// LED pattern state machine
// ============================
enum LedPhase : uint8_t { LED_STEADY = 0, LED_ON_PULSE = 1, LED_OFF_PULSE = 2, LED_PAUSE = 3 };

LedPhase ledPhase = LED_STEADY;
uint8_t  ledTargetBlinks = 0;  // 0 => steady mode
uint8_t  ledBlinksDone   = 0;
uint32_t ledPhaseStartMs = 0;

static void resetLedPattern() {
  ledPhase = LED_STEADY;
  ledTargetBlinks = 0;
  ledBlinksDone = 0;
  ledPhaseStartMs = 0;
}

static uint8_t desiredBlinksForGroup(uint8_t group) {
  if (group == GROUP_WORK) return 2;
  if (group == GROUP_AUTO) return 3;
  return 0; // default group = steady ON
}

static void tickLed() {
  bool connected = compositeHID.isConnected();
  uint32_t now = millis();

  // Rule: if not connected, LED OFF.
  if (!connected) {
    setLed(false);
    resetLedPattern();
    return;
  }

  // Connected: decide target pattern from group
  uint8_t desired = desiredBlinksForGroup(actionGroup);

  // If group changed (or we were reset), re-init LED state
  if (desired != ledTargetBlinks && ledTargetBlinks != 0) {
    // (Handled on group-change too, but this makes it robust)
    resetLedPattern();
  }
  ledTargetBlinks = desired;

  // Group 1: steady ON
  if (ledTargetBlinks == 0) {
    setLed(true);
    ledPhase = LED_STEADY;
    return;
  }

  // Groups 2/3: blink N times, then pause 2s, repeat.
  // During pause, LED stays ON (connected indicator), with brief blink interruptions.
  if (ledPhase == LED_STEADY) {
    ledPhase = LED_ON_PULSE;
    ledBlinksDone = 0;
    ledPhaseStartMs = now;
    setLed(true);
    return;
  }

  if (ledPhase == LED_ON_PULSE) {
    if ((now - ledPhaseStartMs) >= LED_BLINK_MS) {
      ledPhase = LED_OFF_PULSE;
      ledPhaseStartMs = now;
      setLed(false);
    }
    return;
  }

  if (ledPhase == LED_OFF_PULSE) {
    if ((now - ledPhaseStartMs) >= LED_BLINK_MS) {
      ledBlinksDone++;

      if (ledBlinksDone >= ledTargetBlinks) {
        ledPhase = LED_PAUSE;
        ledPhaseStartMs = now;
        setLed(true); // ON during the 2s delay
      } else {
        ledPhase = LED_ON_PULSE;
        ledPhaseStartMs = now;
        setLed(true);
      }
    }
    return;
  }

  // LED_PAUSE
  if ((now - ledPhaseStartMs) >= LED_PAUSE_MS) {
    ledPhase = LED_ON_PULSE;
    ledBlinksDone = 0;
    ledPhaseStartMs = now;
    setLed(true);
  }
}

// ---------- helpers ----------
static inline bool rawPressed(int idx) {
  return digitalRead(btnPin[idx]) == PRESSED_LEVEL;
}

// debounced press edge: true exactly once per press
bool pressedEdge(int idx) {
  uint32_t now = millis();
  uint8_t raw = rawPressed(idx) ? 1 : 0;

  if (raw != btnLastRead[idx]) {
    btnLastRead[idx] = raw;
    btnLastChangeMs[idx] = now;
  }

  if ((now - btnLastChangeMs[idx]) >= DEBOUNCE_MS) {
    if (btnStable[idx] != raw) {
      btnStable[idx] = raw;
      if (btnStable[idx] == 1) return true;
    }
  }
  return false;
}

// Send Ctrl + <key> using modifier-mask path.
void sendCtrlChord(uint8_t keyCode) {
  if (!compositeHID.isConnected()) return;

  keyboard->resetKeys();

  keyboard->modifierKeyPress(KEY_MOD_LCTRL);
  keyboard->keyPress(keyCode);
  delay(10);
  keyboard->keyRelease(keyCode);
  keyboard->modifierKeyRelease(KEY_MOD_LCTRL);

  keyboard->resetKeys();
}

// ============================
// Mouse run (non-blocking circle move)
// ============================
void startMouseRunOnce() {
  mouseRunActive  = true;
  mouseRunStartMs = millis();
  mouseLastTickMs = 0;
  Serial.println("Mouse run: START");
}

void tickMouseRun() {
  if (!mouseRunActive) return;
  if (!compositeHID.isConnected()) return;

  uint32_t now = millis();

  if (now - mouseRunStartMs >= MOUSE_RUN_MS) {
    mouseRunActive = false;
    Serial.println("Mouse run: DONE");
    return;
  }

  if (mouseLastTickMs != 0 && (now - mouseLastTickMs) < MOUSE_TICK_MS) return;
  mouseLastTickMs = now;

  int8_t x = (int8_t)lroundf(cosf((float)now / 1000.0f) * 10.0f);
  int8_t y = (int8_t)lroundf(sinf((float)now / 1000.0f) * 10.0f);

  mouse->mouseMove(x, y);
  mouse->sendMouseReport();
}

// With AutoReport(false), we MUST send reports manually.
static void doMouseClickLogical(uint8_t logicalButton) {
  mouse->mousePress(logicalButton);
  mouse->sendMouseReport();
  delay(3);
  mouse->mouseRelease(logicalButton);
  mouse->sendMouseReport();
}

// ============================
// Typing helpers (ASCII -> HID)
// ============================
static inline void tapKey(uint8_t keyCode, uint16_t msDown = 8) {
  keyboard->keyPress(keyCode);
  delay(msDown);
  keyboard->keyRelease(keyCode);
  delay(4);
}

static inline void tapEnter() {
  tapKey(KEY_ENTER);
}

static inline void tapWin() {
  // META = Windows/GUI key in your keymap
  keyboard->modifierKeyPress(KEY_MOD_LMETA);
  delay(8);
  keyboard->modifierKeyRelease(KEY_MOD_LMETA);
  delay(60); // allow Windows search UI to appear
}

// Supports: letters (a-z/A-Z), digits (0-9), space.
static bool asciiToHid(char c, uint8_t &keyCode, bool &needsShift) {
  needsShift = false;

  if (c >= 'a' && c <= 'z') {
    keyCode = (uint8_t)(KEY_A + (c - 'a'));
    return true;
  }

  if (c >= 'A' && c <= 'Z') {
    keyCode = (uint8_t)(KEY_A + (c - 'A'));
    needsShift = true;
    return true;
  }

  if (c >= '1' && c <= '9') {
    keyCode = (uint8_t)(KEY_1 + (c - '1'));
    return true;
  }
  if (c == '0') {
    keyCode = KEY_0;
    return true;
  }

  if (c == ' ') {
    keyCode = KEY_SPACE;
    return true;
  }

  return false;
}

static void typeChar(char c) {
  uint8_t keyCode = 0;
  bool shift = false;

  if (!asciiToHid(c, keyCode, shift)) return;

  if (shift) keyboard->modifierKeyPress(KEY_MOD_LSHIFT);
  tapKey(keyCode, 6);
  if (shift) keyboard->modifierKeyRelease(KEY_MOD_LSHIFT);
}

static void typeText(const char* s) {
  if (!compositeHID.isConnected()) return;

  keyboard->resetKeys();
  for (const char* p = s; *p; ++p) {
    typeChar(*p);
    delay(6);
  }
  keyboard->resetKeys();
}

static void winSearchAndEnter(const char* query) {
  if (!compositeHID.isConnected()) return;

  tapWin();
  typeText(query);
  tapEnter();
}

static void work_btn1_cmd_ssh() {
  winSearchAndEnter("cmd");
  delay(180);
  typeText("ssh CB37");
  tapEnter();
}

// ============================
// AUTO mode helpers
// ============================
static void stopAutoMode() {
  autoMode = AUTO_OFF;
  autoLastTickMs = 0;
}

static void toggleAutoMode(uint8_t desiredMode) {
  if (autoMode == desiredMode) {
    stopAutoMode();
    Serial.println("AUTO: OFF");
  } else {
    autoMode = desiredMode;
    autoLastTickMs = 0;

    Serial.print("AUTO: ");
    switch (autoMode) {
      case AUTO_LCLICK: Serial.println("LEFT CLICK"); break;
      case AUTO_RCLICK: Serial.println("RIGHT CLICK"); break;
      case AUTO_MCLICK: Serial.println("MIDDLE CLICK"); break;
      case AUTO_KEY_E:  Serial.println("KEY 'e'"); break;
      case AUTO_KEY_F:  Serial.println("KEY 'f'"); break;
      default:          Serial.println("UNKNOWN"); break;
    }
  }
}

static void tickAutoMode() {
  if (autoMode == AUTO_OFF) return;
  if (!compositeHID.isConnected()) return;

  uint32_t now = millis();
  if (autoLastTickMs != 0 && (now - autoLastTickMs) < AUTO_TICK_MS) return;
  autoLastTickMs = now;

  if (autoMode == AUTO_LCLICK) {
    doMouseClickLogical(MOUSE_LOGICAL_LEFT_BUTTON);
  } else if (autoMode == AUTO_RCLICK) {
    doMouseClickLogical(MOUSE_LOGICAL_RIGHT_BUTTON);
  } else if (autoMode == AUTO_MCLICK) {
    doMouseClickLogical(MOUSE_LOGICAL_BUTTON_3);
  } else if (autoMode == AUTO_KEY_E) {
    tapKey(KEY_E, 4);
  } else if (autoMode == AUTO_KEY_F) {
    tapKey(KEY_F, 4);
  }
}

// ============================
// Setup / loop
// ============================
void setup() {
  Serial.begin(115200);
  delay(200);

  // LED
  pinMode(LED_BUILTIN, OUTPUT);
  setLed(false);

  // Buttons
  for (int i = 0; i < BTN_COUNT; i++) {
    pinMode(btnPin[i], BUTTON_PULL_MODE);
    btnLastRead[i] = rawPressed(i) ? 1 : 0;
    btnStable[i] = btnLastRead[i];
    btnLastChangeMs[i] = millis();
  }

  // Advertise as keyboard (matches your working sketch)
  bleHostConfig.setHidType(HID_KEYBOARD);

  // Keyboard: auto reports on press/release
  KeyboardConfiguration keyboardConfig;
  keyboardConfig.setAutoReport(true);
  keyboard = new KeyboardDevice(keyboardConfig);

  // Mouse: manual, since we send at our own tick rate
  MouseConfiguration mouseConfig;
  mouseConfig.setAutoReport(false);
  mouse = new MouseDevice(mouseConfig);

  // Composite HID
  compositeHID.addDevice(keyboard);
  compositeHID.addDevice(mouse);
  compositeHID.begin(bleHostConfig);

  Serial.println("Composite HID started.");
  Serial.println("Group -> DEFAULT");
}

void loop() {
  // Always tick background actions (non-blocking)
  tickMouseRun();
  tickAutoMode();
  tickLed();

  // Button 7: disconnect (reboot drops link; host may auto-reconnect)
  if (pressedEdge(6)) {
    Serial.println("Button 7: disconnect -> reboot");
    delay(100);
    ESP.restart();
  }

  // Button 6: cycle group (works even when not connected)
  if (pressedEdge(5)) {
    actionGroup++;
    if (actionGroup > GROUP_AUTO) actionGroup = GROUP_DEFAULT;

    // Switching groups should stop AUTO spam
    stopAutoMode();

    // Reset LED pattern timing immediately on group change
    resetLedPattern();

    Serial.print("Group -> ");
    if (actionGroup == GROUP_DEFAULT) Serial.println("DEFAULT");
    else if (actionGroup == GROUP_WORK) Serial.println("WORK");
    else Serial.println("AUTO");
  }

  // Nothing else should run unless connected
  if (!compositeHID.isConnected()) return;

  // Buttons 1-5: dispatch by group
  if (actionGroup == GROUP_DEFAULT) {
    if (pressedEdge(0)) { sendCtrlChord(KEY_C); Serial.println("B1: Ctrl+C"); }
    if (pressedEdge(1)) { sendCtrlChord(KEY_V); Serial.println("B2: Ctrl+V"); }
    if (pressedEdge(2)) { sendCtrlChord(KEY_X); Serial.println("B3: Ctrl+X"); }
    if (pressedEdge(3)) { sendCtrlChord(KEY_Z); Serial.println("B4: Ctrl+Z"); }
    if (pressedEdge(4)) { if (!mouseRunActive) startMouseRunOnce(); Serial.println("B5: Mouse run"); }
  }
  else if (actionGroup == GROUP_WORK) {
    if (pressedEdge(0)) { work_btn1_cmd_ssh();       Serial.println("B1: Win cmd -> ssh CB37"); }
    if (pressedEdge(1)) { winSearchAndEnter("edge"); Serial.println("B2: Win edge"); }
    if (pressedEdge(2)) { winSearchAndEnter("super");Serial.println("B3: Win super"); }
    if (pressedEdge(3)) { winSearchAndEnter("ard");  Serial.println("B4: Win ard"); }
    if (pressedEdge(4)) { winSearchAndEnter("blue"); Serial.println("B5: Win blue"); }
  }
  else { // GROUP_AUTO
    // Toggle behaviors:
    // B1 left click, B2 right click, B3 middle click, B4 'e', B5 'f'
    if (pressedEdge(0)) { toggleAutoMode(AUTO_LCLICK); }
    if (pressedEdge(1)) { toggleAutoMode(AUTO_RCLICK); }
    if (pressedEdge(2)) { toggleAutoMode(AUTO_MCLICK); }
    if (pressedEdge(3)) { toggleAutoMode(AUTO_KEY_E);  }
    if (pressedEdge(4)) { toggleAutoMode(AUTO_KEY_F);  }
  }
}
