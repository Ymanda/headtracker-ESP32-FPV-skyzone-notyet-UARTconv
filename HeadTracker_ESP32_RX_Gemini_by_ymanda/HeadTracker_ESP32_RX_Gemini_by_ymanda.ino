// RX_Headtracker.ino
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>

// ============================================================================
// AtomRC gimbal RX bridge (drone side) - AUTO SOURCE SELECTION
//  - CRSF/ELRS has priority
//  - ESP-NOW used as fallback if CRSF is lost
//  - Optional local PPM mode
// ============================================================================

// ---------------------------------------------------------------------------
// SECTION A — Servo core & smoothing
// ---------------------------------------------------------------------------
Servo panServo;
Servo tiltServo;

const int PPM_INPUT_PIN = 2;    // PPM signal (mode PPM local)
const int PAN_PIN       = 18;   // Pan servo output
const int TILT_PIN      = 19;   // Tilt servo output

volatile unsigned long ppmChannels[8] = {
  1500,1500,1500,1500,1500,1500,1500,1500
};
volatile int   ppmChannelIndex   = 0;
volatile bool  ppmFrameComplete  = false;
volatile unsigned long lastValidFrame = 0;

const int PAN_CENTER = 90;
const int PAN_MIN    = 0;
const int PAN_MAX    = 180;

const int TILT_CENTER = 90;
const int TILT_MIN    = 45;
const int TILT_MAX    = 135;

int currentPan  = PAN_CENTER;
int currentTilt = TILT_CENTER;
int targetPan   = PAN_CENTER;
int targetTilt  = TILT_CENTER;

const unsigned long PPM_PULSE_MIN  = 900;
const unsigned long PPM_PULSE_MAX  = 2100;
const unsigned long PPM_FRAME_GAP  = 4000;
const unsigned long SIGNAL_TIMEOUT = 500;   // PPM failsafe timeout

int   deadband     = 3;
int   smoothFactor = 5;

const float LINK_SMOOTH_ALPHA = 0.25f;
const unsigned long LOG_INTERVAL_MS = 2000;

// ---------------------------------------------------------------------------
// SECTION B — Input mode selection
// ---------------------------------------------------------------------------
enum InputMode {
  MODE_PPM_LOCAL = 0,
  MODE_RF_AUTO   = 1
};

InputMode inputMode = MODE_RF_AUTO;

// ---------------- ELRS / CRSF UART (RX only) ----------------
HardwareSerial elrsSerial(1);
const int ELRS_UART_TX_PIN = -1;   // not used on RX
const int ELRS_UART_RX_PIN = 16;   // RX from ELRS module -> ESP32
const unsigned long ELRS_UART_BAUD = 420000;
bool elrsReady = false;

// CRSF constants
const uint8_t  CRSF_DEVICE_ADDRESS        = 0xC8;
const uint8_t  CRSF_FRAMETYPE_RC_CHANNELS = 0x16;
const uint8_t  CRSF_RC_PAYLOAD_LEN        = 22;
const uint8_t  CRSF_RC_CHANNEL_COUNT      = 16;
const uint8_t  CRSF_MAX_FRAME_LEN         = 64;

// ---------------- ESP-NOW (RX only) ----------------
bool espNowReady = false;

// Packet commun (ESP-NOW)
struct __attribute__((packed)) ControlPacket {
  uint16_t header;
  uint16_t pan;
  uint16_t tilt;
  uint16_t flags;
  uint16_t checksum;
};

uint16_t packetChecksum(const ControlPacket& pkt) {
  return pkt.header ^ pkt.pan ^ pkt.tilt ^ pkt.flags ^ 0x55AA;
}

float linkPan  = PAN_CENTER;
float linkTilt = TILT_CENTER;

// ---- ACTIVE SOURCE SELECTION ----
enum ActiveSource {
  SOURCE_NONE = 0,
  SOURCE_ESPNOW,
  SOURCE_CRSF
};

ActiveSource   activeSource      = SOURCE_NONE;
unsigned long  lastEspNowTime    = 0;
unsigned long  lastCrsfTime      = 0;
const unsigned long SOURCE_TIMEOUT_MS = 400;

// --- Forward declarations
void IRAM_ATTR ppmInterrupt();
void processPPMFrame();
void updateServos();
void processSerialCommand(String command);
void testServoMovement();

void initEspNowRX();
void initElrsUartRX();
void setInputMode(InputMode newMode, bool announce);
const char* modeName(InputMode m);
const char* sourceName(ActiveSource s);
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
void readCrsfPackets();
void applyCrsfChannels(const uint8_t* payload);
uint8_t crsfCalcCRC(const uint8_t* data, uint8_t len);
int crsfToMicroseconds(uint16_t value);
int microsecondsToDegrees(int microseconds, int degMin, int degMax);

// -----------------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  currentPan  = constrain(PAN_CENTER,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(TILT_CENTER, TILT_MIN, TILT_MAX);
  targetPan   = currentPan;
  targetTilt  = currentTilt;
  panServo.write(currentPan);
  tiltServo.write(currentTilt);

  pinMode(PPM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), ppmInterrupt, CHANGE);

  Serial.println("RX: Auto source selection (CRSF priority, ESP-NOW fallback)");

  setInputMode(inputMode, false);

  Serial.printf("Initial mode: %s\n", modeName(inputMode));
  Serial.println("Commands: 'center', 'debug', 'test', 'mode pwm|auto'");
  delay(200);
}

// -----------------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------------
void loop() {
  bool haveSignal = false;
  unsigned long now = millis();
  static unsigned long lastHeartbeat = 0;

  if (inputMode == MODE_PPM_LOCAL) {
    // PPM local seulement
    if (ppmFrameComplete) {
      processPPMFrame();
      ppmFrameComplete = false;
      lastValidFrame = now;
    }
    haveSignal = (now - lastValidFrame <= SIGNAL_TIMEOUT);
    if (!haveSignal) {
      targetPan  = PAN_CENTER;
      targetTilt = TILT_CENTER;
      activeSource = SOURCE_NONE;
    }
  } else {
    // RF auto : CRSF priorité, sinon ESP-NOW
    readCrsfPackets(); // met à jour lastCrsfTime si frames valides

    bool crsfAlive = (now - lastCrsfTime   <= SOURCE_TIMEOUT_MS);
    bool espAlive  = (now - lastEspNowTime <= SOURCE_TIMEOUT_MS);

    if (crsfAlive) {
      activeSource = SOURCE_CRSF;
    } else if (espAlive) {
      if (activeSource != SOURCE_CRSF) activeSource = SOURCE_ESPNOW;
    } else {
      activeSource = SOURCE_NONE;
    }

    haveSignal = (activeSource != SOURCE_NONE);
    if (!haveSignal) {
      targetPan  = PAN_CENTER;
      targetTilt = TILT_CENTER;
    }
    if (haveSignal) lastValidFrame = now;
  }

  updateServos();

  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    processSerialCommand(cmd);
  }

  if (now - lastHeartbeat >= LOG_INTERVAL_MS) {
    lastHeartbeat = now;
    Serial.printf("[HB] mode=%s src=%s signal=%s pan=%d tilt=%d\n",
                  modeName(inputMode),
                  sourceName(activeSource),
                  haveSignal ? "ok" : "failsafe",
                  targetPan,
                  targetTilt);
  }

  delay(20);
}

// -----------------------------------------------------------------------------
// PPM ISR & decode
// -----------------------------------------------------------------------------
void IRAM_ATTR ppmInterrupt() {
  static unsigned long lastPulseTime = 0;
  unsigned long currentTime = micros();

  bool pinState = digitalRead(PPM_INPUT_PIN);

  if (pinState == HIGH) {
    lastPulseTime = currentTime;   // début de pulse
  } else {
    unsigned long pulseDuration = currentTime - lastPulseTime;

    if (pulseDuration > PPM_FRAME_GAP) {
      ppmChannelIndex = 0;
    } else if (pulseDuration >= PPM_PULSE_MIN &&
               pulseDuration <= PPM_PULSE_MAX) {
      if (ppmChannelIndex < 8) {
        ppmChannels[ppmChannelIndex] = pulseDuration;
        ppmChannelIndex++;
        if (ppmChannelIndex >= 6) {
          ppmFrameComplete = true;
        }
      }
    }
  }
}



void processPPMFrame() {
  // CH5/CH6 -> pan/tilt (comme tu avais)
  unsigned long panPulse  = ppmChannels[4];  // CH5
  unsigned long tiltPulse = ppmChannels[5];  // CH6

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
    Serial.printf("[PPM] Pan: %lu µs -> %d°, Tilt: %lu µs -> %d°\n",
                  panPulse, targetPan, tiltPulse, targetTilt);
  }
}

// -----------------------------------------------------------------------------
// ESP-NOW receive (RX only)
// -----------------------------------------------------------------------------
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len != sizeof(ControlPacket)) return;

  ControlPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.header != 0xA55A) return;
  if (pkt.checksum != packetChecksum(pkt)) return;
  if ((pkt.flags & 0x0001u) == 0) return; // bit 0 = valid PPM

  linkPan  += (pkt.pan  - linkPan)  * LINK_SMOOTH_ALPHA;
  linkTilt += (pkt.tilt - linkTilt) * LINK_SMOOTH_ALPHA;

  targetPan  = constrain(static_cast<int>(linkPan  + 0.5f), PAN_MIN,  PAN_MAX);
  targetTilt = constrain(static_cast<int>(linkTilt + 0.5f), TILT_MIN, TILT_MAX);

  lastEspNowTime = millis();
  lastValidFrame = lastEspNowTime;

  static bool announced = false;
  if (!announced) {
    announced = true;
    Serial.printf("[ESP-NOW] First packet pan=%u tilt=%u\n", pkt.pan, pkt.tilt);
  }
}

void initEspNowRX() {
  if (espNowReady) return;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
#if ESP_IDF_VERSION_MAJOR >= 4
  esp_now_register_recv_cb([](const esp_now_recv_info* info, const uint8_t* data, int len){
    onEspNowRecv(info->src_addr, data, len);
  });
#else
  esp_now_register_recv_cb(onEspNowRecv);
#endif
  espNowReady = true;
  Serial.println("ESP-NOW RX ready.");
}

// -----------------------------------------------------------------------------
// UART (CRSF) receive (RX only)
// -----------------------------------------------------------------------------
void initElrsUartRX() {
  if (elrsReady) return;
  if (ELRS_UART_RX_PIN < 0) {
    Serial.println("ELRS UART RX pin not defined!");
    return;
  }
  elrsSerial.begin(ELRS_UART_BAUD, SERIAL_8N1, ELRS_UART_RX_PIN, ELRS_UART_TX_PIN);
  elrsReady = true;
  Serial.println("ELRS CRSF UART RX ready on GPIO 16.");
}

void readCrsfPackets() {
  if (!elrsReady) initElrsUartRX();
  if (!elrsReady) return;

  static enum { WAIT_ADDR, WAIT_LEN, READ_FRAME } state = WAIT_ADDR;
  static uint8_t frameLen = 0;
  static uint8_t frameIdx = 0;
  static uint8_t frameBuf[CRSF_MAX_FRAME_LEN];

  while (elrsSerial.available()) {
    uint8_t byteIn = elrsSerial.read();
    switch (state) {
      case WAIT_ADDR:
        if (byteIn == CRSF_DEVICE_ADDRESS) state = WAIT_LEN;
        break;
      case WAIT_LEN:
        if (byteIn < 2 || byteIn > CRSF_MAX_FRAME_LEN) {
          state = WAIT_ADDR;
          break;
        }
        frameLen = byteIn;
        frameIdx = 0;
        state    = READ_FRAME;
        break;
      case READ_FRAME:
        if (frameIdx < CRSF_MAX_FRAME_LEN) {
          frameBuf[frameIdx++] = byteIn;
        }
        if (frameIdx >= frameLen) {
          uint8_t type = frameBuf[0];
          if (frameLen >= 2) {
            uint8_t crc  = frameBuf[frameLen - 1];
            uint8_t calc = crsfCalcCRC(frameBuf, frameLen - 1);
            if (calc == crc) {
              uint8_t payloadLen = frameLen - 2;
              if (type == CRSF_FRAMETYPE_RC_CHANNELS &&
                  payloadLen == CRSF_RC_PAYLOAD_LEN) {
                applyCrsfChannels(&frameBuf[1]);
              }
            }
          }
          state = WAIT_ADDR;
        }
        break;
    }
  }
}

// -----------------------------------------------------------------------------
// Servo motion & CLI
// -----------------------------------------------------------------------------
void updateServos() {
  int panDiff  = targetPan  - currentPan;
  int tiltDiff = targetTilt - currentTilt;

  if (abs(panDiff) > 0) {
    currentPan += panDiff / smoothFactor;
    if (abs(panDiff) < smoothFactor) currentPan = targetPan;
  }
  if (abs(tiltDiff) > 0) {
    currentTilt += tiltDiff / smoothFactor;
    if (abs(tiltDiff) < smoothFactor) currentTilt = targetTilt;
  }

  currentPan  = constrain(currentPan,  PAN_MIN,  PAN_MAX);
  currentTilt = constrain(currentTilt, TILT_MIN, TILT_MAX);

  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

void processSerialCommand(String command) {
  command.toLowerCase();
  if (command == "center") {
    targetPan  = PAN_CENTER;
    targetTilt = TILT_CENTER;
    Serial.println("Center");
  } else if (command == "mode" || command == "mode?") {
    Serial.printf("Current mode: %s\n", modeName(inputMode));
  } else if (command.startsWith("mode")) {
    if (command.endsWith("pwm"))  setInputMode(MODE_PPM_LOCAL, true);
    else if (command.endsWith("auto")) setInputMode(MODE_RF_AUTO, true);
    else Serial.println("Use 'mode pwm' or 'mode auto'");
  } else if (command == "debug") {
    for (int i=0;i<8;i++) {
      Serial.printf("CH%d: %luµs\n", i+1, ppmChannels[i]);
    }
    Serial.printf("Current %d %d Target %d %d Source %s\n",
                  currentPan, currentTilt,
                  targetPan, targetTilt,
                  sourceName(activeSource));
  } else if (command == "test") {
    testServoMovement();
  } else {
    Serial.println("Unknown cmd");
  }
}

void testServoMovement() {
  for (int pos = PAN_CENTER; pos <= PAN_MAX; pos += 10) {
    panServo.write(pos); delay(250);
  }
  for (int pos = PAN_MAX; pos >= PAN_MIN; pos -= 10) {
    panServo.write(pos); delay(250);
  }
  for (int pos = PAN_MIN; pos <= PAN_CENTER; pos += 10) {
    panServo.write(pos); delay(250);
  }

  for (int pos = TILT_CENTER; pos <= TILT_MAX; pos += 6) {
    tiltServo.write(pos); delay(250);
  }
  for (int pos = TILT_MAX; pos >= TILT_MIN; pos -= 6) {
    tiltServo.write(pos); delay(250);
  }
  for (int pos = TILT_MIN; pos <= TILT_CENTER; pos += 6) {
    tiltServo.write(pos); delay(250);
  }

  currentPan  = PAN_CENTER;
  currentTilt = TILT_CENTER;
  targetPan   = PAN_CENTER;
  targetTilt  = TILT_CENTER;
  panServo.write(currentPan);
  tiltServo.write(currentTilt);
}

// -----------------------------------------------------------------------------
// Mode helpers & CRSF parsing helpers
// -----------------------------------------------------------------------------
void setInputMode(InputMode newMode, bool announce) {
  inputMode = newMode;
  switch (inputMode) {
    case MODE_RF_AUTO:
      initEspNowRX();
      initElrsUartRX();
      activeSource = SOURCE_NONE;
      break;
    case MODE_PPM_LOCAL:
    default:
      break;
  }
  if (announce) {
    Serial.printf("Input mode set to %s\n", modeName(inputMode));
  }
}

const char* modeName(InputMode m) {
  switch (m) {
    case MODE_PPM_LOCAL: return "PPM local";
    case MODE_RF_AUTO:   return "RF auto (CRSF+ESP-NOW)";
    default:             return "Unknown";
  }
}

const char* sourceName(ActiveSource s) {
  switch (s) {
    case SOURCE_NONE:   return "NONE";
    case SOURCE_ESPNOW: return "ESP-NOW";
    case SOURCE_CRSF:   return "CRSF";
    default:            return "???";
  }
}

void applyCrsfChannels(const uint8_t* payload) {
  uint32_t bitBuffer = 0;
  uint8_t  bitsInBuffer = 0;
  uint16_t channels[CRSF_RC_CHANNEL_COUNT] = {0};
  uint8_t  byteIndex = 0;

  for (int ch = 0; ch < CRSF_RC_CHANNEL_COUNT; ++ch) {
    while (bitsInBuffer < 11 && byteIndex < CRSF_RC_PAYLOAD_LEN) {
      bitBuffer    |= ((uint32_t)payload[byteIndex++]) << bitsInBuffer;
      bitsInBuffer += 8;
    }
    channels[ch] = bitBuffer & 0x7FF;
    bitBuffer   >>= 11;
    bitsInBuffer -= 11;
  }

  int panMicro  = crsfToMicroseconds(channels[0]);
  int tiltMicro = crsfToMicroseconds(channels[1]);
  int panDeg    = microsecondsToDegrees(panMicro,  PAN_MIN,  PAN_MAX);
  int tiltDeg   = microsecondsToDegrees(tiltMicro, TILT_MIN, TILT_MAX);

  linkPan  += (panDeg  - linkPan)  * LINK_SMOOTH_ALPHA;
  linkTilt += (tiltDeg - linkTilt) * LINK_SMOOTH_ALPHA;

  targetPan  = constrain(static_cast<int>(linkPan  + 0.5f), PAN_MIN,  PAN_MAX);
  targetTilt = constrain(static_cast<int>(linkTilt + 0.5f), TILT_MIN, TILT_MAX);

  lastCrsfTime   = millis();
  lastValidFrame = lastCrsfTime;
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

int crsfToMicroseconds(uint16_t value) {
  value = constrain(value, 172, 1811);
  long micro = map(value, 172, 1811, 988, 2012);
  return (int)constrain(micro, 1000L, 2000L);
}

int microsecondsToDegrees(int microseconds, int degMin, int degMax) {
  microseconds = constrain(microseconds, 1000, 2000);
  long deg = map(microseconds, 1000, 2000, degMin, degMax);
  return (int)constrain(deg, degMin, degMax);
}
