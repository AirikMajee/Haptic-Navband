#include <Arduino.h>
#include <ctype.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// =====================================================
// NAVBAND ESP32-S3 — BLE + USB SERIAL + WEARABLE STATES
// Board: ESP32-S3 SuperMini HW-747
// Motor: GPIO 4 (active HIGH)
// WS2812 RGB: GPIO 48
// Serial: 115200 baud
// =====================================================

#define MOTOR_PIN 4
#define RGB_LED_PIN 48
#define NUM_LEDS 1
#define RGB_BRIGHTNESS 50
#define SERIAL_BAUD 115200

static const char* DEVICE_NAME = "NAV_BAND ESP32";
static const char* SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
static const char* CHARACTERISTIC_UUID = "abcdefab-1234-1234-1234-abcdefabcdef";

// Dashboard commands: L R S A D E
// Device states: I=connected idle, T=destination selected,
// N=navigation active, G=GPS fault, B=link fault, H=heartbeat.

Adafruit_NeoPixel pixels(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
BLECharacteristic* commandCharacteristic = nullptr;

volatile char pendingCommand = '\0';
volatile char pendingState = '\0';
volatile unsigned long lastHeartbeatMs = 0;
volatile bool bleConnected = false;
volatile bool usbSessionActive = false;
volatile bool linkWasEstablished = false;
volatile bool linkLost = false;

String serialBuffer;
char operatingState = 'S'; // S=standby before first connection

// Non-blocking fault-alert scheduler
unsigned long alertNextMs = 0;
uint8_t alertPulseIndex = 0;
bool alertMotorIsOn = false;
unsigned long lastLedUpdateMs = 0;

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  static uint8_t lastR = 255, lastG = 255, lastB = 255;
  if (r == lastR && g == lastG && b == lastB) return;
  lastR = r; lastG = g; lastB = b;
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

void motorOn()  { digitalWrite(MOTOR_PIN, HIGH); }
void motorOff() { digitalWrite(MOTOR_PIN, LOW); }

void pulsePattern(uint8_t r, uint8_t g, uint8_t b,
                  uint16_t onMs, uint16_t offMs, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    setRGB(r, g, b);
    motorOn();
    delay(onMs);
    motorOff();
    setRGB(0, 0, 0);
    if (i + 1 < count) delay(offMs);
  }
}

void executeCommand(char command) {
  command = (char)toupper((unsigned char)command);
  Serial.print("NAV COMMAND: ");
  Serial.println(command);

  switch (command) {
    case 'L': // LEFT: 3 short pulses
      pulsePattern(0, 80, 255, 180, 140, 3);
      break;
    case 'R': // RIGHT: 2 long pulses
      pulsePattern(0, 220, 80, 450, 180, 2);
      break;
    case 'S': // STRAIGHT: 1 short pulse
      pulsePattern(0, 220, 220, 120, 0, 1);
      break;
    case 'A': // APPROACH: 2 short pulses
      pulsePattern(255, 100, 0, 120, 120, 2);
      break;
    case 'D': // ARRIVED: 3 long pulses
      pulsePattern(150, 0, 255, 500, 250, 3);
      break;
    case 'E': // Manual alert burst: 5 rapid pulses
      pulsePattern(255, 0, 0, 100, 100, 5);
      break;
  }

  motorOff();
  // Restore the status indication on the next loop pass.
}

void setOperatingState(char nextState) {
  if (nextState == operatingState) return;

  operatingState = nextState;
  alertPulseIndex = 0;
  alertMotorIsOn = false;
  alertNextMs = millis();
  motorOff();

  Serial.print("WEARABLE STATE: ");
  Serial.println(operatingState);

  // One-time state confirmation patterns.
  if (operatingState == 'I') {
    pulsePattern(0, 180, 40, 90, 0, 1);       // Connected: one short pulse
  } else if (operatingState == 'T') {
    pulsePattern(0, 180, 220, 100, 120, 2);   // Destination selected: two pulses
  } else if (operatingState == 'N') {
    pulsePattern(0, 180, 220, 80, 0, 1);      // Guidance started: one short pulse
  }

  motorOff();
}

void processIncomingChar(char c) {
  c = (char)toupper((unsigned char)c);

  if (c == 'H') {
    lastHeartbeatMs = millis();
    linkWasEstablished = true;
    if (!bleConnected) usbSessionActive = true;
    if (linkLost) {
      linkLost = false;
      pendingState = 'I';
      Serial.println("DEVICE HEARTBEAT RESTORED");
    }
    return;
  }

  if (c == 'I' || c == 'T' || c == 'N' || c == 'G') {
    lastHeartbeatMs = millis();
    linkWasEstablished = true;
    if (!bleConnected) usbSessionActive = true;
    linkLost = false;
    pendingState = c;
    return;
  }

  if (c == 'L' || c == 'R' || c == 'S' || c == 'A' || c == 'D' || c == 'E') {
    lastHeartbeatMs = millis();
    linkWasEstablished = true;
    if (!bleConnected) usbSessionActive = true;
    pendingCommand = c;
  }
}

void updateFaultAlert(unsigned long now) {
  const bool gpsAlert = (operatingState == 'G');
  const bool linkAlert = (operatingState == 'B');

  if (!gpsAlert && !linkAlert) {
    alertPulseIndex = 0;
    alertMotorIsOn = false;
    motorOff();
    return;
  }

  const uint16_t onMs = linkAlert ? 100 : 120;
  const uint16_t offMs = linkAlert ? 100 : 140;
  const uint16_t gapMs = linkAlert ? 1500 : 2800;
  const uint8_t pulseCount = linkAlert ? 5 : 3;

  // Red for link fault; amber for GPS fault. Blink continues while fault persists.
  const unsigned long blinkPeriod = linkAlert ? 350 : 650;
  const unsigned long blinkOn = linkAlert ? 180 : 260;
  if ((now % blinkPeriod) < blinkOn) {
    if (linkAlert) setRGB(255, 0, 0);
    else setRGB(255, 90, 0);
  } else {
    setRGB(0, 0, 0);
  }

  if ((long)(now - alertNextMs) < 0) return;

  if (alertMotorIsOn) {
    motorOff();
    alertMotorIsOn = false;
    alertNextMs = now + offMs;
    return;
  }

  if (alertPulseIndex < pulseCount) {
    motorOn();
    alertMotorIsOn = true;
    alertPulseIndex++;
    alertNextMs = now + onMs;
  } else {
    alertPulseIndex = 0;
    alertNextMs = now + gapMs;
  }
}

void updateStatusLED(unsigned long now) {
  if (operatingState == 'G' || operatingState == 'B') return;

  switch (operatingState) {
    case 'S': // Powered, not yet connected: slow blue heartbeat
      if (now % 1600 < 250) setRGB(0, 0, 150);
      else setRGB(0, 0, 0);
      break;
    case 'I': // Connected standby
      setRGB(0, 150, 35);
      break;
    case 'T': // Destination selected
    case 'N': // Navigation active
      setRGB(0, 150, 190);
      break;
    default:
      setRGB(0, 0, 0);
      break;
  }
}

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    // ESP32 Arduino BLE library in this installation returns Arduino String.
    String value = characteristic->getValue();
    for (int i = 0; i < value.length(); i++) {
      processIncomingChar(value.charAt(i));
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleConnected = true;
    usbSessionActive = false;
    linkWasEstablished = true;
    linkLost = false;
    lastHeartbeatMs = millis();
    pendingState = 'I';
    Serial.println("BLE CLIENT CONNECTED");
  }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    if (linkWasEstablished) {
      linkLost = true;
      pendingState = 'B';
      Serial.println("BLE LINK LOST — RED ALERT ENABLED");
    }
    BLEDevice::startAdvertising();
  }
};

void setupBLE() {
  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  commandCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  commandCharacteristic->setCallbacks(new CommandCallbacks());
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.print("BLE ADVERTISING: ");
  Serial.println(DEVICE_NAME);
}

void receiveUSBCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processIncomingChar(serialBuffer.charAt(0));
        serialBuffer = "";
      }
    } else if (isAlpha((unsigned char)c) && serialBuffer.length() < 8) {
      serialBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(800);

  pinMode(MOTOR_PIN, OUTPUT);
  motorOff();

  pixels.begin();
  pixels.setBrightness(RGB_BRIGHTNESS);
  pixels.clear();
  pixels.show();

  Serial.println();
  Serial.println("========================================");
  Serial.println(" NAVBAND | BLE + USB + FAULT MONITORING");
  Serial.println("========================================");
  Serial.println("Motor: GPIO 4 | RGB: GPIO 48");
  Serial.println("USB Serial: 115200 baud");

  setupBLE();
  setOperatingState('S');
  Serial.println("SYSTEM READY");
  Serial.println("Commands: L R S A D E | States: I T N G B | Heartbeat: H");
}

void loop() {
  receiveUSBCommands();

  // BLE callback updates pending state; apply it safely in the main loop.
  char state = '\0';
  noInterrupts();
  state = pendingState;
  pendingState = '\0';
  interrupts();
  if (state != '\0') setOperatingState(state);

  // A heartbeat is sent by the dashboard every 1.5 seconds.
  // If a previously established BLE/USB link stops responding, latch link fault.
  const unsigned long now = millis();
  if (linkWasEstablished && !linkLost && (bleConnected || usbSessionActive) &&
      (now - lastHeartbeatMs > 5000)) {
    linkLost = true;
    if (!bleConnected) usbSessionActive = false;
    setOperatingState('B');
    Serial.println("LINK HEARTBEAT TIMEOUT — RED ALERT ENABLED");
  }

  char command = '\0';
  noInterrupts();
  command = pendingCommand;
  pendingCommand = '\0';
  interrupts();
  if (command != '\0') executeCommand(command);

  updateFaultAlert(millis());
  updateStatusLED(millis());
  delay(5);
}
