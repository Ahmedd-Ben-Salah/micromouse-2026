#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

#define IN1        17
#define IN2        19
#define IN3        18
#define IN4        23

#define ENC_R_A    25
#define ENC_R_B    26
#define ENC_L_A    27
#define ENC_L_B    14

#define PIN_SDA    21
#define PIN_SCL    22

#define XSHUT_R    32
#define XSHUT_F     4
#define XSHUT_L    33

#define BTN_SELECT  2
#define BTN_UP     16
#define BTN_DOWN   13
#define BTN_STOP    5

#define PIN_POT    35

#define ADDR_TOF_R  0x30
#define ADDR_TOF_F  0x31
#define ADDR_TOF_L  0x32
#define ADDR_GYRO   0x6A
#define ADDR_OLED   0x3C

const float COUNTS_PER_REV = 418.0f;
const float WHEEL_DIAM_MM  = 31.97f;
const float TRACK_MM       = 80.0f;
const float MM_PER_COUNT   = 3.14159f * WHEEL_DIAM_MM / COUNTS_PER_REV;

int   DEADBAND_R = 25;
int   DEADBAND_L = 25;
float MAX_SPEED  = 1295.0f;

float MAZE_SIZE       = 16.0f;
float MIRROR_LR       = 0.0f;
float CELL_MM         = 186.0f;
float FRONT_CENTRE_MM = 50.0f;
float CENTRE_L_MM     = 75.0f;
float CENTRE_R_MM     = 60.0f;
float TH_SIDE_MM      = 110.0f;
float TH_FRONT_MM     = 160.0f;

float GYRO_SIGN  = -1.0f;
float STEER_SIGN =  1.0f;
float TURN_SIGN  =  1.0f;

float GYRO_SCALE = 0.074f;
float KP_HEAD    = 6.0f;
float K_LAT      = 0.55f;
float KI_LAT     = 0.6f;
float MAX_AIM    = 18.0f;

float MIN_MOVE_MMS  = 150.0f;
float EXPLORE_SPEED = 250.0f;
float RUN_SPEED     = 400.0f;
float MOVE_TIMEOUT_S = 8.0f;

const float KP_VEL = 0.08f, KI_VEL = 0.30f, KD_VEL = 0.001f;
const float SENSOR_AHEAD_MM = 30.0f;
const float MIN_VALID_MM = 15.0f, MAX_VALID_MM = 1500.0f;
const float SUM_TOLERANCE = 30.0f;
const float TH_HYST = 15.0f;
const float GYRO_TRUST = 0.95f;
const uint32_t STALE_MS = 250;
const uint32_t LOOP_US  = 10000;

int  gyroSign()  { return GYRO_SIGN  < 0 ? -1 : 1; }
int  steerSign() { return STEER_SIGN < 0 ? -1 : 1; }
int  turnSign()  { return TURN_SIGN  < 0 ? -1 : 1; }
float sumNominal() { return CENTRE_L_MM + CENTRE_R_MM; }

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool okGyro = false, okOled = false;

struct WallSensor {
  Adafruit_VL53L0X dev;
  int      xshutPin = -1;
  uint8_t  addr = 0x29;
  bool     ok = false, wall = false, healthy = false;
  uint16_t raw = 8190;
  float    filtered = 8190.0f;
  uint32_t stamp = 0;
  uint32_t lastGoodMs = 0, rejects = 0, sameCount = 0;
  float    lastAccepted = 8190.0f;

  bool begin(int pin, uint8_t address) {
    xshutPin = pin; addr = address;
    digitalWrite(xshutPin, HIGH);
    delay(100);
    ok = dev.begin(addr, false, &Wire);
    if (ok) {
      dev.setMeasurementTimingBudgetMicroSeconds(20000);
      dev.startRange();
      lastGoodMs = millis();
    }
    return ok;
  }

  void poll() {
    if (!ok) { healthy = false; return; }
    if (!dev.isRangeComplete()) {
      if (millis() - lastGoodMs > STALE_MS) healthy = false;
      return;
    }
    uint16_t r = dev.readRangeResult();

    if (r == 0 || r > MAX_VALID_MM) {
      raw = 8190; healthy = true; lastGoodMs = millis(); stamp++;
      filtered = (filtered > 2000.0f) ? 8190.0f
                                      : 0.4f * filtered + 0.6f * 8190.0f;
      sameCount = 0;
      return;
    }
    if (r < MIN_VALID_MM) {
      if (++rejects > 10) healthy = false;
      return;
    }
    if (fabs((float)r - lastAccepted) < 0.5f) {
      if (++sameCount > 300) healthy = false;
    } else sameCount = 0;
    lastAccepted = r;

    rejects = 0; raw = r; healthy = true; lastGoodMs = millis(); stamp++;

    float t = (float)raw;
    if      (filtered > 2000.0f)    filtered = t;
    else if (t > filtered + 80.0f)  filtered = 0.25f * filtered + 0.75f * t;
    else if (t < filtered - 150.0f) filtered = filtered - 150.0f;
    else                            filtered = 0.6f * filtered + 0.4f * t;
  }

  void updateWallFlag(float threshold) {
    if (!ok || !healthy) { wall = false; return; }
    if (wall) wall = (filtered < threshold + TH_HYST);
    else      wall = (filtered < threshold - TH_HYST);
  }

  bool usable() { return ok && healthy; }
};

WallSensor sensL, sensF, sensR;

uint8_t  rrIndex = 0;
uint32_t rrStartedMs = 0;

void pollSensors() {
  WallSensor *list[3] = { &sensL, &sensF, &sensR };
  WallSensor *cur = list[rrIndex];
  bool advance = false;

  if (!cur->ok)                          advance = true;
  else if (cur->dev.isRangeComplete()) { cur->poll(); advance = true; }
  else if (millis() - rrStartedMs > 60)  advance = true;

  if (advance) {
    rrIndex = (rrIndex + 1) % 3;
    WallSensor *nxt = list[rrIndex];
    if (nxt->ok) nxt->dev.startRange();
    rrStartedMs = millis();
  }
  sensL.updateWallFlag(TH_SIDE_MM);
  sensF.updateWallFlag(TH_FRONT_MM);
  sensR.updateWallFlag(TH_SIDE_MM);
}

void refreshSensors(uint16_t timeoutMs = 250, uint8_t cycles = 2) {
  uint32_t sL = sensL.stamp, sF = sensF.stamp, sR = sensR.stamp;
  uint8_t nL = 0, nF = 0, nR = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    pollSensors();
    if (sensL.stamp != sL) { sL = sensL.stamp; if (nL < 50) nL++; }
    if (sensF.stamp != sF) { sF = sensF.stamp; if (nF < 50) nF++; }
    if (sensR.stamp != sR) { sR = sensR.stamp; if (nR < 50) nR++; }
    if ((!sensL.ok || nL >= cycles) &&
        (!sensF.ok || nF >= cycles) &&
        (!sensR.ok || nR >= cycles)) return;
    delay(2);
  }
}

volatile long countR = 0, countL = 0;

void IRAM_ATTR isrR() { countR += (digitalRead(ENC_R_B) == digitalRead(ENC_R_A)) ? 1 : -1; }
void IRAM_ATTR isrL() { countL -= (digitalRead(ENC_L_B) == digitalRead(ENC_L_A)) ? 1 : -1; }

struct Button {
  uint8_t pin = 0;
  bool stable = true, lastRead = true, event = false;
  uint32_t changed = 0;
  void begin(uint8_t p) { pin = p; pinMode(pin, INPUT_PULLUP); }
  void update() {
    bool r = digitalRead(pin);
    if (r != lastRead) { lastRead = r; changed = millis(); }
    if (millis() - changed > 50 && r != stable) {
      stable = r;
      if (stable == LOW) event = true;
    }
  }
  bool wasPressed() { if (event) { event = false; return true; } return false; }
};

Button btnSelect, btnUp, btnDown, btnStop;

#define REG_WHO      0x0F
#define REG_CTRL2_G  0x11
#define REG_OUT_G    0x24

float gyroBias = 0.0f;

void gyroWrite(uint8_t r, uint8_t v) {
  Wire.beginTransmission(ADDR_GYRO);
  Wire.write(r); Wire.write(v);
  Wire.endTransmission();
}

uint8_t gyroRead(uint8_t r) {
  Wire.beginTransmission(ADDR_GYRO);
  Wire.write(r);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_GYRO, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

float gyroRaw() {
  Wire.beginTransmission(ADDR_GYRO);
  Wire.write(REG_OUT_G);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ADDR_GYRO, (uint8_t)2);
  if (Wire.available() < 2) return 0.0f;
  int16_t v = Wire.read() | (Wire.read() << 8);
  return gyroSign() * v * GYRO_SCALE;
}

float gyroRate() { return gyroRaw() - gyroBias; }

bool calibrateGyro(int samples) {
  double sum = 0; float mn = 9999, mx = -9999;
  for (int i = 0; i < samples; i++) {
    float r = gyroRaw();
    sum += r;
    if (r < mn) mn = r;
    if (r > mx) mx = r;
    delay(3);
  }
  if (mx - mn > 20.0f) return false;
  gyroBias = sum / samples;
  return true;
}

float heading = 0.0f;
float velR = 0, velL = 0;
long  lastCountR = 0, lastCountL = 0;
float distR = 0, distL = 0;

void resetOdometry() {
  noInterrupts(); countR = 0; countL = 0; interrupts();
  lastCountR = 0; lastCountL = 0;
  distR = 0; distL = 0;
  velR = 0; velL = 0;
}

void updateOdometry(float dt) {
  noInterrupts(); long cr = countR, cl = countL; interrupts();
  long dR = cr - lastCountR, dL = cl - lastCountL;
  lastCountR = cr; lastCountL = cl;

  float mmR = dR * MM_PER_COUNT, mmL = dL * MM_PER_COUNT;
  distR += mmR; distL += mmL;
  velR = mmR / dt; velL = mmL / dt;

  float rGyro = okGyro ? gyroRate() : 0.0f;
  float rEnc  = ((mmL - mmR) / TRACK_MM) * 57.2958f / dt;
  heading += (okGyro ? (GYRO_TRUST * rGyro + (1 - GYRO_TRUST) * rEnc) : rEnc) * dt;
}

float travelled() { return (distR + distL) * 0.5f; }

void setTravelled(float want) {
  float shift = want - travelled();
  distR += shift; distL += shift;
}

float squareUpHeading() {
  float snapped = roundf(heading / 90.0f) * 90.0f;
  float corr = snapped - heading;
  if (fabs(corr) < 25.0f) { heading = snapped; return corr; }
  return 0.0f;
}

int lastPwmR = 0, lastPwmL = 0;

void motorsOff() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
  lastPwmR = 0; lastPwmL = 0;
}

void rightPWM(int p) {
  p = constrain(p, -255, 255);
  lastPwmR = p;
  if (p >= 0) { analogWrite(IN1, p);  analogWrite(IN2, 0); }
  else        { analogWrite(IN1, 0);  analogWrite(IN2, -p); }
}

void leftPWM(int p) {
  p = constrain(p, -255, 255);
  lastPwmL = p;
  if (p >= 0) { analogWrite(IN3, p);  analogWrite(IN4, 0); }
  else        { analogWrite(IN3, 0);  analogWrite(IN4, -p); }
}

struct VelPID {
  float integral = 0, lastErr = 0;
  void reset() { integral = 0; lastErr = 0; }
  int compute(float target, float actual, int deadband, float dt) {
    if (fabs(target) < 1.0f) { reset(); return 0; }
    float err = target - actual;
    integral = constrain(integral + err * dt, -400.0f, 400.0f);
    float deriv = (err - lastErr) / dt;
    lastErr = err;
    float ff  = fabs(target) * ((255.0f - deadband) / MAX_SPEED) + deadband;
    float out = constrain(ff + KP_VEL * err + KI_VEL * integral + KD_VEL * deriv,
                          0.0f, 255.0f);
    return (target >= 0) ? (int)out : -(int)out;
  }
};

VelPID pidR, pidL;

void applySteer(float v, float steer, float dt) {
  steer *= steerSign();
  rightPWM(pidR.compute(v - steer, velR, DEADBAND_R, dt));
  leftPWM (pidL.compute(v + steer, velL, DEADBAND_L, dt));
}

enum WallMode { WM_NONE, WM_BOTH, WM_LEFT, WM_RIGHT, WM_DISTRUST };
WallMode wallMode = WM_NONE;

float    latEst = 0.0f;
bool     latValid = false;
uint32_t latStampL = 0, latStampR = 0, latLastGoodMs = 0;

float headBias = 0.0f;

void latReset()    { latEst = 0; latValid = false; latStampL = latStampR = 0; }
void latResetAll() { latReset(); headBias = 0.0f; }

bool lateralError(float &err) {
  bool wl = sensL.usable() && sensL.wall;
  bool wr = sensR.usable() && sensR.wall;

  if (wl && wr) {

    if (fabs((sensL.filtered + sensR.filtered) - sumNominal()) > SUM_TOLERANCE) {
      wallMode = WM_DISTRUST;
      return false;
    }
    wallMode = WM_BOTH;

    err = ((CENTRE_L_MM - sensL.filtered) + (sensR.filtered - CENTRE_R_MM)) * 0.5f;
    return true;
  }
  if (wl) { wallMode = WM_LEFT;  err = CENTRE_L_MM - sensL.filtered; return true; }
  if (wr) { wallMode = WM_RIGHT; err = sensR.filtered - CENTRE_R_MM; return true; }

  wallMode = WM_NONE;
  return false;
}

void latSeed() {
  float e;
  if (lateralError(e)) { latEst = e; latValid = true; }
  else                 { latEst = 0; latValid = false; }
  latStampL = sensL.stamp;
  latStampR = sensR.stamp;
  latLastGoodMs = millis();
}

float latUpdate(float v, float headErr, float dt) {
  if (latValid) latEst += v * sinf(headErr * 0.017453f) * dt;

  bool newL = sensL.usable() && sensL.wall && sensL.stamp != latStampL;
  bool newR = sensR.usable() && sensR.wall && sensR.stamp != latStampR;
  bool fresh = false;
  float e;

  if (newL || newR) {
    if (lateralError(e)) {
      latEst = latValid ? (0.35f * e + 0.65f * latEst) : e;
      latValid = true; fresh = true;
      latLastGoodMs = millis();
    }
    latStampL = sensL.stamp;
    latStampR = sensR.stamp;
  } else {
    lateralError(e);
  }

  if (latValid && millis() - latLastGoodMs > 400) { latValid = false; latEst = 0; }

  if (fresh) headBias = constrain(headBias + KI_LAT * latEst * 0.05f,
                                  -MAX_AIM, MAX_AIM);

  if (!latValid) return headBias;
  return constrain(K_LAT * latEst + headBias, -MAX_AIM, MAX_AIM);
}

const char *modeName() {
  switch (wallMode) {
    case WM_BOTH:     return "both";
    case WM_LEFT:     return "left";
    case WM_RIGHT:    return "rght";
    case WM_DISTRUST: return "BAD!";
    default:          return "none";
  }
}

#define MAZE_N 16
#define W_N 0x01
#define W_E 0x02
#define W_S 0x04
#define W_W 0x08

uint8_t wallMap[MAZE_N][MAZE_N];
uint8_t seenMap[MAZE_N][MAZE_N];
bool    visited[MAZE_N][MAZE_N];
uint8_t distMap[MAZE_N][MAZE_N];

uint8_t failCount[MAZE_N][MAZE_N][4];
uint16_t ffQueue[MAZE_N * MAZE_N];

int  posX = 0, posY = 0, dir = 0;
int  cellsVisited = 0;
bool goalReached = false;

int  mazeSize() { return constrain((int)(MAZE_SIZE + 0.5f), 4, MAZE_N); }
bool mirrored() { return MIRROR_LR >= 0.5f; }
int  goalLo() { return mazeSize() / 2 - 1; }
int  goalHi() { return mazeSize() / 2; }

bool inGoal(int x, int y) {
  return x >= goalLo() && x <= goalHi() && y >= goalLo() && y <= goalHi();
}
bool cellValid(int x, int y) {
  return x >= 0 && x < mazeSize() && y >= 0 && y < mazeSize();
}

int dxOf(int d) { return (d == 1) ? 1 : (d == 3) ? -1 : 0; }
int dyOf(int d) { return (d == 0) ? 1 : (d == 2) ? -1 : 0; }
uint8_t dirBit(int d) { return (d == 0) ? W_N : (d == 1) ? W_E : (d == 2) ? W_S : W_W; }
int opposite(int d) { return (d + 2) & 3; }

int leftDir()  { return mirrored() ? (dir + 1) & 3 : (dir + 3) & 3; }
int rightDir() { return mirrored() ? (dir + 3) & 3 : (dir + 1) & 3; }

void mazeInit() {
  memset(wallMap, 0, sizeof(wallMap));
  memset(seenMap, 0, sizeof(seenMap));
  memset(visited, 0, sizeof(visited));
  memset(failCount, 0, sizeof(failCount));
  int n = mazeSize();
  for (int i = 0; i < n; i++) {
    wallMap[i][0]   |= W_S; seenMap[i][0]   |= W_S;
    wallMap[i][n-1] |= W_N; seenMap[i][n-1] |= W_N;
    wallMap[0][i]   |= W_W; seenMap[0][i]   |= W_W;
    wallMap[n-1][i] |= W_E; seenMap[n-1][i] |= W_E;
  }
  posX = 0; posY = 0; dir = 0;
  cellsVisited = 0; goalReached = false;
}

void markWall(int x, int y, int d, bool present) {
  if (!cellValid(x, y)) return;
  seenMap[x][y] |= dirBit(d);
  if (present) wallMap[x][y] |= dirBit(d); else wallMap[x][y] &= ~dirBit(d);
  int nx = x + dxOf(d), ny = y + dyOf(d);
  if (!cellValid(nx, ny)) return;
  int od = opposite(d);
  seenMap[nx][ny] |= dirBit(od);
  if (present) wallMap[nx][ny] |= dirBit(od); else wallMap[nx][ny] &= ~dirBit(od);
}

bool hasWall(int x, int y, int d) {
  if (!cellValid(x, y)) return true;
  return wallMap[x][y] & dirBit(d);
}
bool wallKnown(int x, int y, int d) {
  if (!cellValid(x, y)) return true;
  return seenMap[x][y] & dirBit(d);
}
bool onEdge(int x, int y, int d) { return !cellValid(x + dxOf(d), y + dyOf(d)); }

void clearCellWalls(int x, int y) {
  if (!cellValid(x, y)) return;
  for (int d = 0; d < 4; d++) {
    if (onEdge(x, y, d)) continue;
    wallMap[x][y] &= ~dirBit(d);
    seenMap[x][y] &= ~dirBit(d);
    int nx = x + dxOf(d), ny = y + dyOf(d), od = opposite(d);
    wallMap[nx][ny] &= ~dirBit(od);
    seenMap[nx][ny] &= ~dirBit(od);
  }
}

void readWallsHere() {
  refreshSensors(250, 3);
  if (sensF.usable()) markWall(posX, posY, dir,        sensF.wall);
  if (sensL.usable()) markWall(posX, posY, leftDir(),  sensL.wall);
  if (sensR.usable()) markWall(posX, posY, rightDir(), sensR.wall);
  if (!visited[posX][posY]) { visited[posX][posY] = true; cellsVisited++; }
}

void floodFill(bool toGoal, bool knownOnly) {
  for (int x = 0; x < MAZE_N; x++)
    for (int y = 0; y < MAZE_N; y++) distMap[x][y] = 255;

  int head = 0, tail = 0;
  if (toGoal) {
    for (int x = goalLo(); x <= goalHi(); x++)
      for (int y = goalLo(); y <= goalHi(); y++) {
        distMap[x][y] = 0; ffQueue[tail++] = (x << 8) | y;
      }
  } else {
    distMap[0][0] = 0; ffQueue[tail++] = 0;
  }

  while (head < tail) {
    uint16_t c = ffQueue[head++];
    int cx = c >> 8, cy = c & 0xFF;
    uint8_t d = distMap[cx][cy];
    for (int dd = 0; dd < 4; dd++) {
      if (hasWall(cx, cy, dd)) continue;
      if (knownOnly && !wallKnown(cx, cy, dd)) continue;
      int nx = cx + dxOf(dd), ny = cy + dyOf(dd);
      if (!cellValid(nx, ny) || distMap[nx][ny] != 255) continue;
      distMap[nx][ny] = d + 1;
      ffQueue[tail++] = (nx << 8) | ny;
    }
  }
}

int bestDirection() {
  int order[4] = { dir, (dir + 1) & 3, (dir + 3) & 3, (dir + 2) & 3 };
  int best = -1;
  uint8_t bd = distMap[posX][posY];
  for (int i = 0; i < 4; i++) {
    int d = order[i];
    if (hasWall(posX, posY, d)) continue;
    int nx = posX + dxOf(d), ny = posY + dyOf(d);
    if (!cellValid(nx, ny)) continue;
    if (distMap[nx][ny] < bd) { bd = distMap[nx][ny]; best = d; }
  }
  return best;
}

void drawBanner(const char *l1, const char *l2) {
  if (!okOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2); oled.setCursor(0, 8);  oled.println(l1);
  oled.setTextSize(1); oled.setCursor(0, 44); oled.println(l2);
  oled.display();
}

bool userAborted = false;

bool driveStraight(float distanceMm, float speedMms,
                   bool useWalls, bool stopAtFrontWall,
                   float frontStopMm = 50.0f) {
  resetOdometry();
  pidR.reset(); pidL.reset();
  latReset();
  if (useWalls) { refreshSensors(120, 1); latSeed(); }
  userAborted = false;

  if (distanceMm > 0) { rightPWM(80); leftPWM(80); delay(25); }

  float startHeading = roundf(heading / 90.0f) * 90.0f;

  const float ACCEL_MM = 60.0f;
  uint32_t last = micros(), t0 = millis();

  while (true) {
    btnStop.update();
    if (btnStop.wasPressed()) { motorsOff(); userAborted = true; return false; }
    if (millis() - t0 > (uint32_t)(MOVE_TIMEOUT_S * 1000.0f)) {
      motorsOff();
      Serial.printf("drive TIMEOUT after %.0f mm\n", travelled());
      return false;
    }

    uint32_t now = micros();
    if (now - last < LOOP_US) continue;
    float dt = (now - last) / 1000000.0f;
    last = now;

    updateOdometry(dt);
    pollSensors();

    float done = travelled();
    float remaining = distanceMm - done;

    if (useWalls && sensF.usable() && sensF.wall && sensF.filtered < 220.0f) {
      float rf = sensF.filtered - frontStopMm;
      if (rf < remaining) remaining = rf;
    }

    if (stopAtFrontWall && sensF.usable() && sensF.filtered < frontStopMm) break;
    if (remaining <= 0.5f) break;

    float v = speedMms;
    if (done < ACCEL_MM)      v = speedMms * (0.3f + 0.7f * done / ACCEL_MM);
    if (remaining < ACCEL_MM) v = speedMms * (0.3f + 0.7f * remaining / ACCEL_MM);

    if (sensF.usable() && sensF.filtered < 220.0f) {
      float f = (sensF.filtered - frontStopMm) / (220.0f - frontStopMm);
      v *= constrain(f, 0.5f, 1.0f);
    }

    if (useWalls && latValid && fabs(latEst) > 10.0f)
      v *= constrain(1.0f - (fabs(latEst) - 10.0f) / 35.0f, 0.5f, 1.0f);

    if (v < MIN_MOVE_MMS) v = MIN_MOVE_MMS;

    float headErr = heading - startHeading;
    float aim = useWalls ? latUpdate(v, headErr, dt) : 0.0f;

    float steer = KP_HEAD * (headErr + aim);
    float maxSteer = v - MIN_MOVE_MMS * 0.6f;
    if (maxSteer < 20.0f) maxSteer = 20.0f;
    steer = constrain(steer, -maxSteer, maxSteer);

    applySteer(v, steer, dt);
  }

  rightPWM(-70); leftPWM(-70);
  delay(40);
  motorsOff();
  delay(70);
  return true;
}

bool driveBack(float mm) {
  if (mm < 5.0f) return true;
  resetOdometry();
  uint32_t t0 = millis(), last = micros();
  while (travelled() > -mm && millis() - t0 < 2500) {
    uint32_t now = micros();
    if (now - last < LOOP_US) continue;
    float dt = (now - last) / 1000000.0f;
    last = now;
    updateOdometry(dt);
    rightPWM(-90); leftPWM(-90);
    btnStop.update();
    if (btnStop.wasPressed()) { motorsOff(); userAborted = true; return false; }
  }
  motorsOff(); delay(120);
  return true;
}

bool turnInPlace(float degrees) {
  resetOdometry();
  float target = heading + degrees;
  uint32_t last = micros(), t0 = millis(), settled = 0;

  while (true) {
    btnStop.update();
    if (btnStop.wasPressed()) { motorsOff(); userAborted = true; return false; }
    if (millis() - t0 > (uint32_t)(MOVE_TIMEOUT_S * 1000.0f)) {
      motorsOff();
      Serial.printf("turn TIMEOUT: head %.1f target %.1f\n", heading, target);
      return false;
    }
    uint32_t now = micros();
    if (now - last < LOOP_US) continue;
    float dt = (now - last) / 1000000.0f;
    last = now;

    updateOdometry(dt);
    float err = target - heading;

    if (fabs(err) < 4.0f) {
      if (settled == 0) {
        settled = millis();

        int b = 55 * turnSign() * (degrees >= 0 ? 1 : -1);
        rightPWM(-b); leftPWM(b);
        delay(35);
        motorsOff();
      } else if (millis() - settled > 180) break;
    } else settled = 0;

    int pwm = constrain((int)(2.0f * err), -85, 85);
    if (fabs(err) > 5.0f && abs(pwm) < 60) pwm = (pwm > 0 ? 60 : -60);

    rightPWM( pwm * turnSign());
    leftPWM (-pwm * turnSign());
  }
  motorsOff(); delay(80);
  return true;
}

void advancePosition() {
  posX = constrain(posX + dxOf(dir), 0, mazeSize() - 1);
  posY = constrain(posY + dyOf(dir), 0, mazeSize() - 1);
}

bool advanceOneCell(float speed) {
  int fX = posX, fY = posY, fD = dir;

  driveStraight(CELL_MM, speed, true, true, FRONT_CENTRE_MM);
  if (userAborted) return false;
  float moved = travelled();

  refreshSensors(200);
  bool frontWall = sensF.usable() && sensF.wall &&
                   sensF.filtered < FRONT_CENTRE_MM + 30.0f;

  if (moved > CELL_MM * 0.60f) {
    advancePosition();
    markWall(fX, fY, fD, false);
    failCount[fX][fY][fD] = 0;
    if (frontWall) squareUpHeading();
    return true;
  }

  if (frontWall) {
    markWall(fX, fY, fD, true);
    failCount[fX][fY][fD] = 0;
    squareUpHeading();
    return false;
  }

  if (failCount[fX][fY][fD] < 250) failCount[fX][fY][fD]++;
  if (failCount[fX][fY][fD] >= 2) {
    markWall(fX, fY, fD, true);
    Serial.printf("blocked twice at %d,%d dir %d - recording a wall\n", fX, fY, fD);
  }

  if (moved > 30.0f) driveBack(moved);
  return false;
}

bool turnLeft()   { if (!turnInPlace( 90.0f)) return false; dir = leftDir();     squareUpHeading(); return true; }
bool turnRight()  { if (!turnInPlace(-90.0f)) return false; dir = rightDir();    squareUpHeading(); return true; }
bool turnAround() { if (!turnInPlace(180.0f)) return false; dir = opposite(dir); squareUpHeading(); return true; }

bool turnTo(int target) {
  if (target == dir)        return true;
  if (target == rightDir()) return turnRight();
  if (target == leftDir())  return turnLeft();
  return turnAround();
}

int consecutiveCells = 0, bestConsecutive = 0, failures = 0;
uint32_t runStartMs = 0, runElapsedMs = 0;

void statsReset() {
  consecutiveCells = 0; bestConsecutive = 0; failures = 0;
  runStartMs = millis(); runElapsedMs = 0;
}

bool navigate(bool toGoal, float speed, int maxSteps,
              bool mapAsWeGo, bool knownOnly) {
  int steps = 0, stuck = 0;

  while (steps++ < maxSteps) {
    btnStop.update();
    if (btnStop.wasPressed()) { motorsOff(); return false; }

    bool arrived = toGoal ? inGoal(posX, posY) : (posX == 0 && posY == 0);
    if (arrived) return true;

    floodFill(toGoal, knownOnly);
    if (knownOnly && distMap[posX][posY] == 255) floodFill(toGoal, false);

    int want = bestDirection();

    if (want < 0) {

      stuck++;
      char l2[24];
      snprintf(l2, sizeof(l2), stuck == 1 ? "re-reading" : "forget r=%d", stuck - 1);
      drawBanner("STUCK", l2);
      delay(250);

      if (stuck == 1) {
        readWallsHere();
      } else if (stuck <= 5) {
        int r = stuck - 1;
        for (int x = posX - r; x <= posX + r; x++)
          for (int y = posY - r; y <= posY + r; y++)
            if (cellValid(x, y)) clearCellWalls(x, y);
        readWallsHere();
      } else {
        Serial.println("still boxed in after widening - giving up");
        return false;
      }
      continue;
    }
    stuck = 0;

    if (!turnTo(want)) return false;

    if (advanceOneCell(speed)) {
      consecutiveCells++;
      if (consecutiveCells > bestConsecutive) bestConsecutive = consecutiveCells;
      if (mapAsWeGo) readWallsHere();
    } else {
      if (userAborted) { motorsOff(); return false; }
      consecutiveCells = 0; failures++;
      readWallsHere();
    }

    char l2[24];
    snprintf(l2, sizeof(l2), "%d,%d d%d  %d cells", posX, posY, dir, cellsVisited);
    drawBanner(toGoal ? "TO GOAL" : "RETURN", l2);
  }
  return false;
}

Preferences prefs;

struct Param { const char *name; float *value; float lo, hi; const char *key; };

Param params[] = {
  { "Maze size",       &MAZE_SIZE,          4,  16,  "size"  },
  { "Mirror L/R",      &MIRROR_LR,          0,   1,  "mirr"  },
  { "Gyro sign",       &GYRO_SIGN,         -1,   1,  "sgyr"  },
  { "Steer sign",      &STEER_SIGN,        -1,   1,  "sstr"  },
  { "Turn sign",       &TURN_SIGN,         -1,   1,  "strn"  },
  { "Cell pitch mm",   &CELL_MM,          150, 210,  "cell"  },
  { "Front centre mm", &FRONT_CENTRE_MM,   20, 120,  "front" },
  { "Centre L mm",     &CENTRE_L_MM,       20, 130,  "cenl"  },
  { "Centre R mm",     &CENTRE_R_MM,       20, 130,  "cenr"  },
  { "Side thresh mm",  &TH_SIDE_MM,        40, 200,  "thsid" },
  { "Gyro scale",      &GYRO_SCALE,     0.04f, 0.12f,"gscl"  },
  { "Head gain",       &KP_HEAD,         2.0f, 15.0f,"kph"   },
  { "Aim deg per mm",  &K_LAT,           0.1f,  1.5f,"klat"  },
  { "Aim learn rate",  &KI_LAT,          0.0f,  2.0f,"kilat" },
  { "Max aim deg",     &MAX_AIM,         6.0f, 30.0f,"maim"  },
  { "Min move mm/s",   &MIN_MOVE_MMS,     100, 220,  "vmin"  },
  { "Explore speed",   &EXPLORE_SPEED,    150, 450,  "vexp"  },
  { "Run speed",       &RUN_SPEED,        200, 700,  "vrun"  },
  { "Move timeout s",  &MOVE_TIMEOUT_S,     3,  20,  "tmo"   }
};
const int PARAM_COUNT = sizeof(params) / sizeof(params[0]);

int  paramIdx = 0, potAnchor = -1;
bool potActive = false;

void settingsLoad() {
  prefs.begin("mm", true);
  for (int i = 0; i < PARAM_COUNT; i++)
    *params[i].value = prefs.getFloat(params[i].key, *params[i].value);
  prefs.end();
}
void settingsSave() {
  prefs.begin("mm", false);
  for (int i = 0; i < PARAM_COUNT; i++)
    prefs.putFloat(params[i].key, *params[i].value);
  prefs.end();
}

enum State {
  ST_IDLE, ST_CALIBRATE, ST_DIAG, ST_SIGNS, ST_MAP, ST_TUNE, ST_PROBE,
  ST_T_CELL, ST_T_WALLS, ST_T_TURN, ST_T_DRIVE, ST_T_CORRIDOR,
  ST_EXPLORE, ST_RETURN, ST_SPEEDRUN
};
State state = ST_IDLE;

const char *menuItems[] = {
  "EXPLORE",
  "SPEED RUN",
  "Return to start",
  "Calibrate gyro",
  "Diagnostics",
  "Test: signs",
  "Show map",
  "Settings",
  "Test: cell probe",
  "Test: corridor",
  "Test: one cell",
  "Test: read walls",
  "Test: turn 90",
  "Test: drive 500"
};
const int MENU_COUNT = 14;
int menuIndex = 0;
uint32_t lastOledMs = 0;

int potSpeed() { return map(analogRead(PIN_POT), 0, 4095, 150, 700); }

void drawMenu() {
  if (!okOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.printf("SPD%4d  %d,%d d%d", potSpeed(), posX, posY, dir);
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  int first = max(0, min(menuIndex - 2, MENU_COUNT - 5));
  for (int i = 0; i < 5 && first + i < MENU_COUNT; i++) {
    int idx = first + i;
    oled.setCursor(0, 13 + i * 10);
    oled.print(idx == menuIndex ? ">" : " ");
    oled.print(menuItems[idx]);
  }
  oled.display();
}

void drawDiag() {
  if (!okOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.printf("DIAG      mode %s", modeName());
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  oled.setCursor(0, 13);
  oled.printf("L%4.0f%c F%4.0f%c R%4.0f%c\n",
              sensL.filtered, sensL.wall ? '#' : '.',
              sensF.filtered, sensF.wall ? '#' : '.',
              sensR.filtered, sensR.wall ? '#' : '.');
  oled.printf("sum %4.0f want %4.0f\n",
              sensL.filtered + sensR.filtered, sumNominal());
  oled.printf("lat %5.1f bias %5.1f\n", latEst, headBias);
  oled.printf("head %6.1f  %d,%d d%d\n", heading, posX, posY, dir);
  oled.printf("enc L%5ld R%5ld", countL, countR);
  oled.display();
}

void drawSigns() {
  if (!okOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("SIGN CHECK");
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  oled.setCursor(0, 13);
  oled.printf("turn CW by hand:\n");
  oled.printf("  head %6.1f  want -\n", heading);
  oled.printf("push to LEFT wall:\n");
  oled.printf("  lat  %6.1f  want +\n", latEst);
  oled.printf("mode %s   L%3.0f R%3.0f", modeName(),
              sensL.filtered, sensR.filtered);
  oled.display();
}

void drawMap() {
  if (!okOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.printf("MAP %d,%d d%d  %dx%d", posX, posY, dir, mazeSize(), mazeSize());
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  const int R = 3, cs = 7, ox = 4, oy = 13;
  for (int j = -R; j <= R; j++)
    for (int i = -R; i <= R; i++) {
      int cx = posX + i, cy = posY + j;
      if (!cellValid(cx, cy)) continue;
      int px = ox + (i + R) * cs, py = oy + (R - j) * cs;
      if (visited[cx][cy]) oled.drawPixel(px + 3, py + 3, SSD1306_WHITE);
      if (hasWall(cx, cy, 0)) oled.drawFastHLine(px, py, cs, SSD1306_WHITE);
      if (hasWall(cx, cy, 2)) oled.drawFastHLine(px, py + cs - 1, cs, SSD1306_WHITE);
      if (hasWall(cx, cy, 3)) oled.drawFastVLine(px, py, cs, SSD1306_WHITE);
      if (hasWall(cx, cy, 1)) oled.drawFastVLine(px + cs - 1, py, cs, SSD1306_WHITE);
    }
  oled.fillRect(ox + R * cs + 2, oy + R * cs + 2, 3, 3, SSD1306_WHITE);
  oled.setCursor(72, 24); oled.printf("N up");
  oled.setCursor(72, 36); oled.printf("v %d", cellsVisited);
  oled.setCursor(72, 48); oled.printf("%s", goalReached ? "GOAL" : "-");
  oled.display();
}

void doIdle() {
  if (btnUp.wasPressed())   menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT;
  if (btnDown.wasPressed()) menuIndex = (menuIndex + 1) % MENU_COUNT;

  if (btnSelect.wasPressed()) {
    switch (menuIndex) {
      case 0:  state = ST_EXPLORE;     break;
      case 1:  state = ST_SPEEDRUN;    break;
      case 2:  state = ST_RETURN;      break;
      case 3:  state = ST_CALIBRATE;   break;
      case 4:  state = ST_DIAG;        break;
      case 5:  state = ST_SIGNS;       break;
      case 6:  state = ST_MAP;         break;
      case 7:  state = ST_TUNE; paramIdx = 0; potAnchor = -1; potActive = false; break;
      case 8:  state = ST_PROBE;       break;
      case 9:  state = ST_T_CORRIDOR;  break;
      case 10: state = ST_T_CELL;      break;
      case 11: state = ST_T_WALLS;     break;
      case 12: state = ST_T_TURN;      break;
      case 13: state = ST_T_DRIVE;     break;
    }
    return;
  }
  if (millis() - lastOledMs > 200) { lastOledMs = millis(); drawMenu(); }
}

void doCalibrate() {
  drawBanner("HOLD STILL", "calibrating gyro");
  delay(400);
  if (calibrateGyro(500)) {
    heading = 0;
    latResetAll();
    drawBanner("GYRO OK", "bias cleared");
  } else {
    drawBanner("MOVED!", "hold still, retry");
  }
  delay(1300);
  state = ST_IDLE;
}

void doLiveScreen(void (*draw)()) {
  pollSensors();
  if (millis() - lastOledMs > 120) {
    lastOledMs = millis();
    latSeed();
    draw();
  }
}

void doTune() {
  pollSensors();
  bool changed = false;
  if (btnSelect.wasPressed()) { paramIdx = (paramIdx + 1) % PARAM_COUNT; changed = true; }
  if (btnUp.wasPressed())     { paramIdx = (paramIdx + PARAM_COUNT - 1) % PARAM_COUNT; changed = true; }
  if (btnDown.wasPressed())   { paramIdx = (paramIdx + 1) % PARAM_COUNT; changed = true; }

  int pot = analogRead(PIN_POT);
  if (potAnchor < 0 || changed) { potAnchor = pot; potActive = false; }
  if (abs(pot - potAnchor) > 120) potActive = true;

  Param &pm = params[paramIdx];
  if (potActive) *pm.value = pm.lo + (pot / 4095.0f) * (pm.hi - pm.lo);

  if (millis() - lastOledMs < 120) return;
  lastOledMs = millis();
  if (!okOled) return;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.printf("SETTINGS  %d/%d", paramIdx + 1, PARAM_COUNT);
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  oled.setCursor(0, 13);
  oled.println(pm.name);
  oled.setTextSize(2);
  oled.setCursor(0, 25);
  if      (pm.value == &MAZE_SIZE)  oled.printf("%d", mazeSize());
  else if (pm.value == &MIRROR_LR)  oled.printf("%s", mirrored() ? "LEFT" : "RIGHT");
  else if (pm.value == &GYRO_SIGN)  oled.printf("%+d", gyroSign());
  else if (pm.value == &STEER_SIGN) oled.printf("%+d", steerSign());
  else if (pm.value == &TURN_SIGN)  oled.printf("%+d", turnSign());
  else if (pm.value == &GYRO_SCALE) oled.printf("%.4f", *pm.value);
  else if (pm.hi <= 2.5f)           oled.printf("%.2f", *pm.value);
  else if (pm.hi <= 20.0f)          oled.printf("%.1f", *pm.value);
  else                              oled.printf("%.0f", *pm.value);
  oled.setTextSize(1);
  oled.setCursor(0, 46);
  oled.printf("L%4.0f R%4.0f F%4.0f\n", sensL.filtered, sensR.filtered, sensF.filtered);
  oled.setCursor(0, 56);
  oled.println(potActive ? "GRN next   YEL save" : "turn pot to edit");
  oled.display();
}

void doProbe() {
  pollSensors();
  if (millis() - lastOledMs < 150) return;
  lastOledMs = millis();
  if (!okOled) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("CELL PROBE");
  oled.drawFastHLine(0, 9, 128, SSD1306_WHITE);
  oled.setCursor(0, 13);
  oled.println("park centred, wall ahead");
  oled.setTextSize(2);
  oled.setCursor(0, 23);
  oled.printf("F%3.0f", sensF.filtered);
  oled.setTextSize(1);
  oled.setCursor(0, 44);
  oled.printf("L %3.0f -> Centre L\n", sensL.filtered);
  oled.printf("R %3.0f -> Centre R", sensR.filtered);
  oled.display();
}

void doTestCorridor() {
  drawBanner("CORRIDOR", "1000 mm, walls on");
  delay(700);
  driveStraight(1000.0f, potSpeed(), true, false);
  char l2[24];
  snprintf(l2, sizeof(l2), "lat %.0f bias %.0f", latEst, headBias);
  drawBanner("DONE", l2);
  delay(2000);
  state = ST_IDLE;
}

void doTestDrive() {
  drawBanner("DRIVE", "500 mm, no walls");
  delay(700);
  driveStraight(500.0f, potSpeed(), false, false);
  char l2[24];
  snprintf(l2, sizeof(l2), "moved %.0f mm", travelled());
  drawBanner("DONE", l2);
  delay(2200);
  state = ST_IDLE;
}

void doTestCell() {
  drawBanner("ONE CELL", "advancing");
  delay(600);
  bool ok = advanceOneCell(potSpeed());
  char l2[24];
  snprintf(l2, sizeof(l2), "%d,%d d%d %s", posX, posY, dir, ok ? "ok" : "blocked");
  drawBanner(ok ? "DONE" : "BLOCKED", l2);
  delay(1800);
  state = ST_IDLE;
}

void doTestWalls() {
  drawBanner("READ WALLS", "hold still");
  delay(300);
  readWallsHere();
  char l2[24];
  snprintf(l2, sizeof(l2), "L%c F%c R%c at %d,%d",
           sensL.wall ? '#' : '.', sensF.wall ? '#' : '.',
           sensR.wall ? '#' : '.', posX, posY);
  drawBanner("WALLS", l2);
  delay(2200);
  state = ST_IDLE;
}

void doTestTurn() {
  drawBanner("TURN", "90 right");
  delay(600);
  float before = heading;
  bool ok = turnRight();
  char l2[24];
  snprintf(l2, sizeof(l2), "d%d  change %.0f", dir, heading - before);
  drawBanner(ok ? "DONE" : "STOPPED", l2);
  delay(2000);
  state = ST_IDLE;
}

void doExplore() {
  drawBanner("EXPLORE", "flood fill");
  delay(700);

  mazeInit();
  heading = 0;
  statsReset();
  readWallsHere();

  navigate(true, EXPLORE_SPEED, 500, true, false);
  goalReached = inGoal(posX, posY);
  runElapsedMs = millis() - runStartMs;

  char l2[24];
  snprintf(l2, sizeof(l2), "%d cells best %d", cellsVisited, bestConsecutive);
  drawBanner(goalReached ? "GOAL!" : "STOPPED", l2);
  Serial.printf("explore: goal %d cells %d best %d fail %d %.1fs\n",
                goalReached, cellsVisited, bestConsecutive, failures,
                runElapsedMs / 1000.0f);
  delay(2200);

  if (goalReached) {

    drawBanner("RETURN", "mapping new ground");
    delay(500);
    navigate(false, EXPLORE_SPEED, 500, true, false);
    turnTo(0);
    floodFill(true, true);
    int known = distMap[0][0];
    floodFill(true, false);
    snprintf(l2, sizeof(l2), "path %d  %s", known,
             known == distMap[0][0] ? "OPTIMAL" : "approx");
    drawBanner("READY", l2);
    delay(2200);
  }
  state = ST_IDLE;
}

void doReturn() {
  drawBanner("RETURN", "to start");
  delay(600);
  statsReset();
  bool ok = navigate(false, EXPLORE_SPEED, 500, true, false);
  if (ok) turnTo(0);
  drawBanner(ok ? "AT START" : "STOPPED", "");
  delay(1500);
  state = ST_IDLE;
}

void doSpeedRun() {
  floodFill(true, true);
  if (!goalReached || distMap[posX][posY] == 255) {
    drawBanner("NO MAP", "explore first");
    delay(1500);
    state = ST_IDLE;
    return;
  }
  drawBanner("SPEED RUN", "best known path");
  delay(800);
  statsReset();
  bool ok = navigate(true, RUN_SPEED, 400, false, true);
  runElapsedMs = millis() - runStartMs;
  char l2[24];
  snprintf(l2, sizeof(l2), "%.2f s  best %d", runElapsedMs / 1000.0f, bestConsecutive);
  drawBanner(ok ? "FINISHED" : "STOPPED", l2);
  Serial.printf("speed run: ok %d %.2f s\n", ok, runElapsedMs / 1000.0f);
  delay(3000);
  state = ST_IDLE;
}

void setup() {

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  pinMode(XSHUT_R, OUTPUT); digitalWrite(XSHUT_R, LOW);
  pinMode(XSHUT_F, OUTPUT); digitalWrite(XSHUT_F, LOW);
  pinMode(XSHUT_L, OUTPUT); digitalWrite(XSHUT_L, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== MICROMOUSE clean build ===");

  btnSelect.begin(BTN_SELECT);
  btnUp.begin(BTN_UP);
  btnDown.begin(BTN_DOWN);
  btnStop.begin(BTN_STOP);

  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  delay(80);

  okOled = oled.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
  if (okOled) drawBanner("MICROMOUSE", "starting up");

  bool a = sensR.begin(XSHUT_R, ADDR_TOF_R);
  bool b = sensF.begin(XSHUT_F, ADDR_TOF_F);
  bool c = sensL.begin(XSHUT_L, ADDR_TOF_L);

  uint8_t id = gyroRead(REG_WHO);
  okGyro = (id == 0x69 || id == 0x6A);
  if (okGyro) { gyroWrite(REG_CTRL2_G, 0x4C); delay(150); }

  Serial.printf("ToF R%s F%s L%s | gyro %s (0x%02X) | OLED %s\n",
                a ? "+" : "-", b ? "+" : "-", c ? "+" : "-",
                okGyro ? "ok" : "--", id, okOled ? "ok" : "--");

  settingsLoad();

  if (okGyro) {
    drawBanner("HOLD STILL", "auto-calibrating");
    delay(400);
    if (!calibrateGyro(400)) Serial.println("startup calibration rejected");
    heading = 0;
    latResetAll();
  }

  mazeInit();
  resetOdometry();
  motorsOff();

  rrIndex = 0;
  rrStartedMs = millis();
  refreshSensors(600, 4);

  char line[32];
  snprintf(line, sizeof(line), "ToF %d/3  cell %.0f",
           (a ? 1 : 0) + (b ? 1 : 0) + (c ? 1 : 0), CELL_MM);
  drawBanner("READY", line);
  delay(1400);
  Serial.println("ready - motors OFF\n");
}

void loop() {
  btnSelect.update();
  btnUp.update();
  btnDown.update();
  btnStop.update();

  static uint32_t lastTick = micros();
  uint32_t now = micros();
  bool parked = (state == ST_IDLE || state == ST_DIAG || state == ST_SIGNS ||
                 state == ST_MAP  || state == ST_TUNE || state == ST_PROBE);
  if (parked) {
    if (now - lastTick >= LOOP_US) {
      updateOdometry((now - lastTick) / 1000000.0f);
      lastTick = now;
    }
  } else lastTick = now;

  if (state != ST_IDLE && btnStop.wasPressed()) {
    motorsOff();
    if (state == ST_TUNE) {
      settingsSave();
      drawBanner("SAVED", "settings stored");
      delay(900);
    }
    state = ST_IDLE;
    return;
  }

  switch (state) {
    case ST_IDLE:        doIdle();                 break;
    case ST_CALIBRATE:   doCalibrate();            break;
    case ST_DIAG:        doLiveScreen(drawDiag);   break;
    case ST_SIGNS:       doLiveScreen(drawSigns);  break;
    case ST_MAP:         doLiveScreen(drawMap);    break;
    case ST_TUNE:        doTune();                 break;
    case ST_PROBE:       doProbe();                break;
    case ST_T_CORRIDOR:  doTestCorridor();         break;
    case ST_T_DRIVE:     doTestDrive();            break;
    case ST_T_CELL:      doTestCell();             break;
    case ST_T_WALLS:     doTestWalls();            break;
    case ST_T_TURN:      doTestTurn();             break;
    case ST_EXPLORE:     doExplore();              break;
    case ST_RETURN:      doReturn();               break;
    case ST_SPEEDRUN:    doSpeedRun();             break;
    default:             state = ST_IDLE;          break;
  }
}
