// TX_Headtracker.ino
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// ============================================================================
// Skyzone → AtomRC TX bridge (goggles side) - 3-POS SWITCH CONTROL
// ELRS module remains powered at all times.
// ============================================================================

// -----------------------------------------------------------------------------
// USER CONFIGURATION — TX (GOGGLES SIDE)
// -----------------------------------------------------------------------------
const uint8_t ESP_NOW_PEER_MAC[6] = {0xF4, 0x65, 0x0B, 0x49, 0x12, 0x94};

// 3-position hardware mode selector (common -> GND)
// POS1 (LOCAL)  -> drives MODE_SW_PIN_A LOW (GPIO32)
// POS3 (ELRS)   -> drives MODE_SW_PIN_B LOW (GPIO33)
// Middle (ESP-NOW) -> both pins HIGH (no contact to GND)
const int MODE_SW_PIN_A = 32;
const int MODE_SW_PIN_B = 33;

// ---------------------------------------------------------------------------
// SECTION 1 — Inputs & globals
// ---------------------------------------------------------------------------
Servo panServo;
Servo tiltServo;

// *** PPM sur GPIO25 ***
const int PPM_INPUT_PIN = 25;
const int PAN_PIN       = 18;
const int TILT_PIN      = 19;

volatile unsigned long ppmChannels[8] = {
  1500,1500,1500,1500,1500,1500,1500,1500
};
volatile int   ppmChannelIndex     = 0;
volatile bool  ppmFrameComplete    = false;
volatile unsigned long lastValidFrame = 0;

// sondes debug PPM
volatile unsigned long lastPulseDur = 0;
volatile unsigned long isrHits      = 0;
volatile int           lastPpmIndex = 0;

const int PAN_CENTER = 90;
const int PAN_MIN    = 0;
const int PAN_MAX    = 180;
const int TILT_CENTER = 90;
const int TILT_MIN    = 30;
const int TILT_MAX    = 150;

float currentPan  = PAN_CENTER;
float currentTilt = TILT_CENTER;
float targetPan   = PAN_CENTER;
float targetTilt  = TILT_CENTER;

const unsigned long PPM_PULSE_MIN  = 900;
const unsigned long PPM_PULSE_MAX  = 2100;
const unsigned long PPM_FRAME_GAP  = 4000;
const unsigned long PPM_TIMEOUT    = 1000;

int   deadband = 3;

// GAIN type "Gemini_smooth"
const float PAN_GAIN  = 0.20f;
const float TILT_GAIN = 0.20f;

const float WIRELESS_SMOOTH_ALPHA = 0.35f;

enum OutputMode {
  MODE_WIRED_PWM = 0,
  MODE_CRSF_ELRS = 1,
  MODE_ESPNOW    = 2
};

OutputMode outputMode = MODE_CRSF_ELRS;

// ---------------- ELRS / CRSF UART (TX only) --------------
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN = 17;   // vers module ELRS
const int ELRS_UART_RX_PIN = -1;   // pas de RX côté TX
const unsigned long ELRS_UART_BAUD = 420000;
bool elrsReady = false;

// CRSF constants
const uint8_t  CRSF_DEVICE_ADDRESS        = 0xC8;
const uint8_t  CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
const uint8_t  CRSF_RC_PAYLOAD_LEN        = 22;
const uint8_t  CRSF_RC_CHANNEL_COUNT      = 16;
const uint16_t CRSF_CHANNEL_MID           = 992;

// ---------------- ESP-NOW (TX only) ----------------
bool    espNowReady = false;
uint8_t espNowPeer[6] = {0};
const unsigned long WIRELESS_SEND_INTERVAL_MS = 10;
unsigned long lastWirelessSend = 0;

struct __attribute__((packed)) ControlPacket {
  uint16_t header;
  uint16_t pan;
  uint16_t tilt;
  uint16_t flags;
  uint16_t checksum;
};

float wirelessPan  = PAN_CENTER;
float wirelessTilt = TILT_CENTER;

// --- Forward declarations
void processSerialCommand(String command);
void testServoMovement();
void IRAM_ATTR ppmInterrupt();
void setOutputMode(OutputMode newMode, bool announce);
void handleOutputs(bool ppmValid);
void transmitWirelessTargets(bool ppmValid);
void initEspNow();
void initElrsUart();
const char* modeName(OutputMode mode);
uint16_t packetChecksum(const ControlPacket& pkt);
void processPPMFrame();
void updateServos();
void sendCrsfRcFrame(int panDeg, int tiltDeg, bool valid);
uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len);
void encodeCrsfChannels(const uint16_t* channels, uint8_t* payload);
uint16_t microsecondsToCrsf(int microseconds);
int degreesToMicroseconds(int degrees, int degMin, int degMax);
OutputMode readHardwareMode();

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  memcpy(espNowPeer, ESP_NOW_PEER_MAC, sizeof(espNowPeer));

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  currentPan  = constrain(currentPan,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);
  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);

  Serial.println("TX: Head Tracker (goggles)");

  // PPM input
  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  // mode switch pins
  pinMode(MODE_SW_PIN_A, INPUT_PULLUP);
  pinMode(MODE_SW_PIN_B, INPUT_PULLUP);

  // read switch once at boot
  outputMode = readHardwareMode();
  setOutputMode(outputMode, true);

  Serial.printf("ESP-NOW peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                espNowPeer[0], espNowPeer[1], espNowPeer[2],
                espNowPeer[3], espNowPeer[4], espNowPeer[5]);

  Serial.println("Commands: center, debug, test, deadband X");
  delay(200);
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  // hardware switch overrides
  static OutputMode lastHwMode = outputMode;
  OutputMode hwMode = readHardwareMode();
  if (hwMode != lastHwMode) {
    lastHwMode = hwMode;
    setOutputMode(hwMode, true);
  }

  // PPM frame ready ?
  if (ppmFrameComplete) {
    processPPMFrame();
    ppmFrameComplete = false;
    lastValidFrame = millis();
  }

  // failsafe si plus de PPM
  if (millis() - lastValidFrame > PPM_TIMEOUT) {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
  }

  bool ppmValid = (millis() - lastValidFrame) <= PPM_TIMEOUT;
  handleOutputs(ppmValid);

  // CLI série
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    processSerialCommand(command);
  }

  // Debug PPM
  static unsigned long lastDbg = 0;
  unsigned long now = millis();
  if (now - lastDbg > 1000) {   // toutes les secondes
    lastDbg = now;

    noInterrupts();
    unsigned long hits = isrHits;
    unsigned long dur  = lastPulseDur;
    int           idx  = lastPpmIndex;
    isrHits = 0; // on remet à zéro le compteur pour voir l'activité
    interrupts();

    Serial.printf("[PPM DBG] isrHits=%lu lastDur=%lu us lastIdx=%d\n",
                  hits, dur, idx);
  }

  delay(20);
}

// -----------------------------------------------------------------------------
// PPM acquisition (ISR) — **version du vieux code qui marchait**
// -----------------------------------------------------------------------------
// Mesure de la largeur du pulse HIGH (entre front montant et descendant)
void IRAM_ATTR ppmInterrupt() {
  static unsigned long lastPulseTime = 0;
  unsigned long currentTime = micros();

  bool pinState = digitalRead(PPM_INPUT_PIN);

  if (pinState == HIGH) {
    // front montant : début de pulse
    lastPulseTime = currentTime;
  } else {
    // front descendant : fin de pulse -> durée HIGH
    unsigned long pulseDuration = currentTime - lastPulseTime;

    // debug
    lastPulseDur = pulseDuration;
    isrHits++;
    lastPpmIndex = ppmChannelIndex;

    if (pulseDuration > PPM_FRAME_GAP) {
      // grande pause -> nouvelle trame
      ppmChannelIndex = 0;
      // on ne met pas frameComplete ici, on lève le flag
      // dès qu'on a assez de canaux
    } else if (pulseDuration >= PPM_PULSE_MIN &&
               pulseDuration <= PPM_PULSE_MAX) {
      if (ppmChannelIndex < 8) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;

        // dès qu'on a CH0..CH5 on considère la frame exploitable
        if (ppmChannelIndex >= 6) {
          ppmFrameComplete = true;
        }
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Utilisation de la frame PPM
// -----------------------------------------------------------------------------
void processPPMFrame() {
  // mapping qui marchait : CH5 / CH6
  unsigned long panPulse  = ppmChannels[5];  // CH6 -> PAN
  unsigned long tiltPulse = ppmChannels[4];  // CH5 -> TILT

  if (panPulse < PPM_PULSE_MIN  || panPulse > PPM_PULSE_MAX)  panPulse  = 1500;
  if (tiltPulse < PPM_PULSE_MIN || tiltPulse > PPM_PULSE_MAX) tiltPulse = 1500;

  int newPan  = map(panPulse,  1000, 2000, PAN_MIN,  PAN_MAX);
  int newTilt = map(tiltPulse, 1000, 2000, TILT_MIN, TILT_MAX);

  if (abs(newPan  - PAN_CENTER)  < deadband) newPan  = PAN_CENTER;
  if (abs(newTilt - TILT_CENTER) < deadband) newTilt = TILT_CENTER;

  targetPan  = constrain(newPan,  PAN_MIN,  PAN_MAX);
  targetTilt = constrain(newTilt, TILT_MIN, TILT_MAX);

  static int frameCount = 0;
  if (frameCount++ % 50 == 0) {
    Serial.printf("Pan: %lu µs -> %d°, Tilt: %lu µs -> %d°\n",
                  panPulse, (int)targetPan, tiltPulse, (int)targetTilt);
  }
}

// -----------------------------------------------------------------------------
// Servo smoothing (for local PWM mode) — gain 0.2 / 0.2
// -----------------------------------------------------------------------------
void updateServos() {
  float panError  = targetPan  - currentPan;
  float tiltError = targetTilt - currentTilt;

  currentPan  += panError  * PAN_GAIN;
  currentTilt += tiltError * TILT_GAIN;

  if (fabs(panError)  < 0.5f) currentPan  = targetPan;
  if (fabs(tiltError) < 0.5f) currentTilt = targetTilt;

  currentPan  = constrain(currentPan,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);

  panServo.write((int)currentPan);
  tiltServo.write((int)currentTilt);
}

// -----------------------------------------------------------------------------
// Serial & helper functions (inchangés)
// -----------------------------------------------------------------------------
void processSerialCommand(String command) {
  command.toLowerCase();
  if (command == "center") {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
    Serial.println("Center");
  } else if (command == "debug") {
    for (int i=0;i<8;i++) {
      Serial.printf("CH%d %lu\n", i+1, ppmChannels[i]);
    }
    Serial.printf("Mode: %s Target: %d %d\n",
                  modeName(outputMode), (int)targetPan, (int)targetTilt);
  } else if (command == "test") {
    testServoMovement();
  } else if (command.startsWith("deadband")) {
    int v = command.substring(command.indexOf(' ')+1).toInt();
    deadband = v;
    Serial.printf("Deadband=%d\n", deadband);
  } else {
    Serial.println("Unknown");
  }
}

void testServoMovement() {
  for (int pos = PAN_CENTER; pos <= PAN_MAX; pos += 10) {
    panServo.write(pos); delay(200);
  }
  for (int pos = PAN_MAX; pos >= PAN_MIN; pos -= 10) {
    panServo.write(pos); delay(200);
  }
  for (int pos = PAN_MIN; pos <= PAN_CENTER; pos += 10) {
    panServo.write(pos); delay(200);
  }
  for (int pos = TILT_CENTER; pos <= TILT_MAX; pos += 6) {
    tiltServo.write(pos); delay(200);
  }
  for (int pos = TILT_MAX; pos >= TILT_MIN; pos -= 6) {
    tiltServo.write(pos); delay(200);
  }
  for (int pos = TILT_MIN; pos <= TILT_CENTER; pos += 6) {
    tiltServo.write(pos); delay(200);
  }
}

// -----------------------------------------------------------------------------
// Outputs (ESP-NOW or CRSF) — identiques à ton code
// -----------------------------------------------------------------------------
void handleOutputs(bool ppmValid) {
  if (outputMode == MODE_WIRED_PWM) {
    updateServos();
    return;
  }
  if (millis() - lastWirelessSend >= WIRELESS_SEND_INTERVAL_MS) {
    transmitWirelessTargets(ppmValid);
    lastWirelessSend = millis();
  }
}

void transmitWirelessTargets(bool ppmValid) {
  wirelessPan  += (targetPan  - wirelessPan)  * WIRELESS_SMOOTH_ALPHA;
  wirelessTilt += (targetTilt - wirelessTilt) * WIRELESS_SMOOTH_ALPHA;

  ControlPacket pkt;
  pkt.header = 0xA55A;
  pkt.pan    = constrain(static_cast<int>(wirelessPan  + 0.5f), PAN_MIN,  PAN_MAX);
  pkt.tilt   = constrain(static_cast<int>(wirelessTilt + 0.5f), TILT_MIN, TILT_MAX);
  pkt.flags  = ppmValid ? 0x0001 : 0x0000;
  pkt.checksum = packetChecksum(pkt);

  if (outputMode == MODE_ESPNOW) {
    if (!espNowReady) initEspNow();
    if (espNowReady) {
      esp_now_send(espNowPeer, reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    }
  } else if (outputMode == MODE_CRSF_ELRS) {
    sendCrsfRcFrame(pkt.pan, pkt.tilt, ppmValid);
  }
}

void setOutputMode(OutputMode newMode, bool announce) {
  outputMode = newMode;
  switch (outputMode) {
    case MODE_CRSF_ELRS:
      initElrsUart();
      break;
    case MODE_ESPNOW:
      initEspNow();
      break;
    case MODE_WIRED_PWM:
    default:
      break;
  }
  if (announce) {
    Serial.printf("Output mode set to %s\n", modeName(outputMode));
  }
}

void initElrsUart() {
  if (elrsReady) return;
  if (ELRS_UART_TX_PIN < 0) {
    Serial.println("ELRS UART TX pin not defined!");
    return;
  }
  elrsSerial.begin(ELRS_UART_BAUD, SERIAL_8N1, ELRS_UART_RX_PIN, ELRS_UART_TX_PIN);
  elrsReady = true;
  Serial.println("ELRS CRSF UART (TX) ready on GPIO 17.");
}

void initEspNow() {
  if (espNowReady) return;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, espNowPeer, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
  }
  espNowReady = true;
  Serial.println("ESP-NOW link (TX) ready.");
}

const char* modeName(OutputMode mode) {
  switch (mode) {
    case MODE_WIRED_PWM: return "Wired PWM";
    case MODE_CRSF_ELRS: return "CRSF / ELRS";
    case MODE_ESPNOW:    return "ESP-NOW";
    default:             return "Unknown";
  }
}

uint16_t packetChecksum(const ControlPacket& pkt) {
  return pkt.header ^ pkt.pan ^ pkt.tilt ^ pkt.flags ^ 0x55AA;
}

// -----------------------------------------------------------------------------
// CRSF helpers (inchangés)
// -----------------------------------------------------------------------------
void sendCrsfRcFrame(int panDeg, int tiltDeg, bool valid) {
  if (!elrsReady) initElrsUart();
  if (!elrsReady) return;

  uint16_t channels[CRSF_RC_CHANNEL_COUNT];
  for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; ++i) {
    channels[i] = CRSF_CHANNEL_MID;
  }

  if (valid) {
    int panMicro  = degreesToMicroseconds(panDeg,  PAN_MIN,  PAN_MAX);
    int tiltMicro = degreesToMicroseconds(tiltDeg, TILT_MIN, TILT_MAX);
    channels[0] = microsecondsToCrsf(panMicro);
    channels[1] = microsecondsToCrsf(tiltMicro);
  }

  uint8_t payload[CRSF_RC_PAYLOAD_LEN] = {0};
  encodeCrsfChannels(channels, payload);

  const uint8_t frameLen = 1 + CRSF_RC_PAYLOAD_LEN + 1;
  uint8_t frame[2 + frameLen];
  frame[0] = CRSF_DEVICE_ADDRESS;
  frame[1] = frameLen;
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;
  memcpy(&frame[3], payload, CRSF_RC_PAYLOAD_LEN);
  frame[3 + CRSF_RC_PAYLOAD_LEN] = crsfCalcCRC(&frame[2], 1 + CRSF_RC_PAYLOAD_LEN);

  elrsSerial.write(frame, sizeof(frame));
}

uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len) {
  const uint8_t POLY = 0xD5;
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80) crc = (crc << 1) ^ POLY;
      else            crc <<= 1;
    }
  }
  return crc;
}

void encodeCrsfChannels(const uint16_t* channels, uint8_t* payload) {
  uint32_t bitBuffer    = 0;
  uint8_t  bitsInBuffer = 0;
  uint8_t  byteIndex    = 0;
  for (int ch = 0; ch < CRSF_RC_CHANNEL_COUNT; ++ch) {
    bitBuffer    |= ((uint32_t)channels[ch] & 0x7FF) << bitsInBuffer;
    bitsInBuffer += 11;
    while (bitsInBuffer >= 8) {
      payload[byteIndex++] = bitBuffer & 0xFF;
      bitBuffer   >>= 8;
      bitsInBuffer -= 8;
    }
  }
  if (bitsInBuffer > 0 && byteIndex < CRSF_RC_PAYLOAD_LEN) {
    payload[byteIndex++] = bitBuffer & 0xFF;
  }
  while (byteIndex < CRSF_RC_PAYLOAD_LEN) {
    payload[byteIndex++] = 0;
  }
}

uint16_t microsecondsToCrsf(int microseconds) {
  microseconds = constrain(microseconds, 988, 2012);
  long value   = map(microseconds, 988, 2012, 172, 1811);
  return (uint16_t)constrain(value, 172L, 1811L);
}

int degreesToMicroseconds(int degrees, int degMin, int degMax) {
  degrees = constrain(degrees, degMin, degMax);
  long micro = map(degrees, degMin, degMax, 1000, 2000);
  return (int)constrain(micro, 1000L, 2000L);
}

// -----------------------------------------------------------------------------
// Hardware mode selector helper (inchangé)
// -----------------------------------------------------------------------------
OutputMode readHardwareMode() {
  bool aLow = (digitalRead(MODE_SW_PIN_A) == LOW);
  bool bLow = (digitalRead(MODE_SW_PIN_B) == LOW);

  // A = LOW, B = HIGH -> LOCAL bench
  if (aLow && !bLow) return MODE_WIRED_PWM;

  // A = HIGH, B = LOW -> ELRS / CRSF
  if (!aLow && bLow) return MODE_CRSF_ELRS;

  // both HIGH -> middle -> ESP-NOW
  return MODE_ESPNOW;
}
