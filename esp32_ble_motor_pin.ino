/*
  ESP32 DevKit V1 - 2 động cơ 775 (2x BTS7960) + JGB37-555 + SRF04 qua Web BLE
  Thư viện: NimBLE-Arduino 2.x (h2zero)

  Bảo mật: ghép nối bằng PIN 6 số (LE Secure Connections + MITM + bonding).
  ESP32 không có màn hình nên dùng PIN cố định; máy tính/điện thoại sẽ hiện ô nhập PIN
  khi ghép nối lần đầu. Characteristic ghi lệnh yêu cầu liên kết đã xác thực.

  Giao thức (văn bản):
    M:<L>,<R> | J:1/0 | E | RESET | LIM:<n> | ?
    ESP32 -> Web: ERR:AUTH | ERR:ESTOP | S:L,R,J,LIM,E | D:<cm> | D:--
*/

#include <NimBLEDevice.h>
#include <Preferences.h>

// ================== CẤU HÌNH ==================
const char*    DEVICE_NAME = "ESP32-Motor";
const uint32_t BLE_PIN     = 123456;   // <-- ĐỔI PIN (6 chữ số, 000000-999999)
const bool     CLEAR_BONDS = false;    // đặt true, nạp 1 lần để xoá mọi thiết bị đã ghép (nên làm sau khi đổi PIN)

const int PIN_L_RPWM = 25;   // D25
const int PIN_L_LPWM = 26;   // D26
const int PIN_R_RPWM = 27;   // D27
const int PIN_R_LPWM = 14;   // D14
const int PIN_JGB    = 23;   // D23, HIGH = chạy
const int PIN_TRIG   = 12;   // D12
const int PIN_ECHO   = 13;   // D13  *** SRF04 ra 5V -> phải chia áp xuống 3.3V ***

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// ===============================================

const int PWM_PINS[4] = { PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM };

NimBLEServer* server = nullptr;
NimBLECharacteristic* txChar = nullptr;
Preferences prefs;

volatile bool connected = false;
volatile bool authed = false;       // true khi liên kết đã ghép nối bằng PIN
volatile bool estop = false;
volatile bool jgbOn = false;
volatile int  reqL = 0, reqR = 0;
volatile int  limitPct = 100;
volatile bool kickClient = false;
volatile uint16_t connHandle = 0;

String outQ[10];
volatile int qHead = 0, qTail = 0;
void qPush(const String& s) {
  int next = (qHead + 1) % 10;
  if (next == qTail) return;
  outQ[qHead] = s;
  qHead = next;
}

// ---------- PWM (tương thích core 2.x và 3.x) ----------
void pwmInit() {
  for (int i = 0; i < 4; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PWM_PINS[i], PWM_FREQ, PWM_BITS);
    ledcWrite(PWM_PINS[i], 0);
#else
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PWM_PINS[i], i);
    ledcWrite(i, 0);
#endif
  }
}
void pwmSet(int i, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PINS[i], duty);
#else
  ledcWrite(i, duty);
#endif
}
void driveMotor(int idxR, int idxL, int pct) {
  int duty = abs(pct) * 255 / 100;
  if (pct > 0)      { pwmSet(idxL, 0); pwmSet(idxR, duty); }
  else if (pct < 0) { pwmSet(idxR, 0); pwmSet(idxL, duty); }
  else              { pwmSet(idxR, 0); pwmSet(idxL, 0); }
}

int effL() { return estop ? 0 : constrain((int)reqL, -(int)limitPct, (int)limitPct); }
int effR() { return estop ? 0 : constrain((int)reqR, -(int)limitPct, (int)limitPct); }

void applyOutputs() {
  driveMotor(0, 1, effL());
  driveMotor(2, 3, effR());
  digitalWrite(PIN_JGB, (jgbOn && !estop) ? HIGH : LOW);
}

void stopAll() {
  reqL = reqR = 0;
  jgbOn = false;
  applyOutputs();
}

void pushState() {
  qPush("S:" + String(effL()) + "," + String(effR()) + "," +
        String(jgbOn && !estop ? 1 : 0) + "," + String((int)limitPct) + "," +
        String(estop ? 1 : 0));
}

// ---------- Xử lý lệnh ----------
void handleCmd(String s) {
  s.trim();
  if (s.length() == 0) return;

  if (!authed) { qPush("ERR:AUTH"); return; }

  if (s == "E") {
    estop = true;
    stopAll();
    pushState();
  } else if (s == "RESET") {
    estop = false;
    stopAll();
    pushState();
  } else if (s.startsWith("LIM:")) {
    limitPct = constrain(s.substring(4).toInt(), 10, 100);
    prefs.putInt("lim", limitPct);
    applyOutputs();
    pushState();
  } else if (s == "?") {
    pushState();
  } else if (estop) {
    qPush("ERR:ESTOP");
  } else if (s.startsWith("M:")) {
    int c = s.indexOf(',');
    if (c > 2) {
      reqL = constrain(s.substring(2, c).toInt(), -100, 100);
      reqR = constrain(s.substring(c + 1).toInt(), -100, 100);
      applyOutputs();
      pushState();
    }
  } else if (s.startsWith("J:")) {
    jgbOn = (s.substring(2).toInt() == 1);
    applyOutputs();
    pushState();
  }
}

// ---------- NimBLE callbacks (API 2.x) ----------
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    connected = true;
    authed = false;
    connHandle = connInfo.getConnHandle();
    // Chủ động yêu cầu ghép nối/mã hoá ngay khi kết nối
    NimBLEDevice::startSecurity(connHandle);
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    connected = false;
    authed = false;
    estop = false;
    stopAll();                       // mất kết nối -> dừng an toàn
  }
  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    if (connInfo.isEncrypted() && connInfo.isAuthenticated()) {
      authed = true;
    } else {
      authed = false;
      kickClient = true;             // ghép nối thất bại (sai PIN) -> ngắt
    }
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue v = c->getValue();
    handleCmd(String(v.c_str()));
  }
};

// ---------- SRF04 ----------
float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (us == 0) return -1;
  return us * 0.0343f / 2.0f;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_JGB, OUTPUT);
  digitalWrite(PIN_JGB, LOW);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pwmInit();

  prefs.begin("motor", false);
  limitPct = constrain(prefs.getInt("lim", 100), 10, 100);

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(185);

  // --- Bảo mật: ghép nối bằng PIN ---
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);   // ESP32 "hiển thị" PIN, máy kia nhập
  NimBLEDevice::setSecurityAuth(true, true, true);          // bonding, MITM, Secure Connections
  NimBLEDevice::setSecurityPasskey(BLE_PIN);
  if (CLEAR_BONDS) NimBLEDevice::deleteAllBonds();

  server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(SERVICE_UUID);
  txChar = svc->createCharacteristic(
      TX_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_AUTHEN);
  NimBLECharacteristic* rx = svc->createCharacteristic(
      RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_AUTHEN);
  rx->setCallbacks(new RxCB());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(DEVICE_NAME);
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();
  Serial.println("San sang. Ten BLE: " + String(DEVICE_NAME));
}

void sendNow(const String& s) {
  txChar->setValue((const uint8_t*)s.c_str(), s.length());
  txChar->notify();
  delay(5);
}

void loop() {
  while (qTail != qHead) {
    if (connected) sendNow(outQ[qTail]);
    qTail = (qTail + 1) % 10;
  }

  if (kickClient) {
    kickClient = false;
    delay(100);
    server->disconnect(connHandle);
  }

  static unsigned long lastD = 0;
  if (millis() - lastD >= 200) {
    lastD = millis();
    float d = readDistanceCm();
    if (connected && authed) {
      sendNow(d < 0 ? String("D:--") : "D:" + String(d, 1));
    }
  }
}
