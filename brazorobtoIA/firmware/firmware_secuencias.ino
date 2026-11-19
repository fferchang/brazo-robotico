#include <errno.h>
#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

static const uint16_t MOVE_SPEED = 450;
static const uint8_t MOVE_ACC = 45;
// Orden `sync`: hasta cinco motores del brazo (M1..M5) en un solo SyncWritePosEx,
// con velocidades proporcionales para que lleguen juntos.
// Nunca 0: en el STS3215 una velocidad 0 significa "maxima".
static const uint16_t SYNC_MIN_SPEED = 20;
static const uint8_t SYNC_MAX_MOTORS = 5;
static const long SYNC_MAX_DURATION_MS = 10000;
// Velocidad de M1..M5 (`speed max <pasos/s> [acc]`): al arrancar, la de siempre. La PC
// la cambia al conectar segun config.json. ARM_SPEED_HARD_MAX es el techo absoluto.
static const uint16_t ARM_SPEED_MIN_SETTING = 100;
static const uint16_t ARM_SPEED_HARD_MAX = 1000;
static const uint8_t ARM_ACC_MIN_SETTING = 10;
static const uint8_t ARM_ACC_MAX_SETTING = 150;
uint16_t armMaxSpeed = MOVE_SPEED;
uint8_t armAcc = MOVE_ACC;
// Supervisor de la fuente (`power guard`): cada POWER_SAMPLE_INTERVAL_MS lee la
// corriente y el voltaje de los seis servos. Con el voltaje mas bajo bajo el umbral
// lento o la corriente total sobre el suyo, M1..M5 van a la mitad de velocidad
// (todos por igual: la ruta no cambia) hasta GUARD_SCALE_MIN_PCT; con el umbral de
// corte durante GUARD_HOLD_SAMPLES lecturas seguidas, el brazo se frena donde esta.
// Vuelve de a GUARD_SCALE_STEP_UP_PCT tras GUARD_RECOVER_SAMPLES lecturas normales.
// Apagado al arrancar: lo prende la app al conectar.
static const unsigned long POWER_SAMPLE_INTERVAL_MS = 100;
static const uint8_t GUARD_HOLD_SAMPLES = 2;
static const uint8_t GUARD_RECOVER_SAMPLES = 10;
static const uint8_t GUARD_SCALE_MIN_PCT = 25;
static const uint8_t GUARD_SCALE_STEP_UP_PCT = 25;
static const float CURRENT_MA_PER_UNIT = 6.5f;
bool powerGuard = false;
int guardSlowDv = 95;
int guardHoldDv = 80;
long guardSlowMa = 3000;
long guardHoldMa = 4500;
uint8_t speedScalePct = 100;
uint8_t guardHardSamples = 0;
uint8_t guardGoodSamples = 0;
// Ultimo destino ordenado a cada motor del brazo y su velocidad sin escalar, para
// reenviarlos con otra velocidad si el supervisor la cambia en pleno movimiento.
int armGoal[6] = {0};
uint16_t armBaseSpeed[6] = {0};
bool armGoalValid[6] = {false};
unsigned long armGoalAtMs[6] = {0};
// Una lectura de la fuente (va antes de las funciones: Arduino declara sus prototipos ahi).
struct PowerReading {
  int raw[7];             // por ID 1..6: corriente cruda (6,5 mA por unidad), -1 sin lectura
  long totalRaw;
  int minDv;             // decimas de voltio, -1 sin lectura
  uint8_t minId;
};

// Solo se reenvia un destino ordenado hace menos de esto: nunca uno viejo.
static const unsigned long ARM_GOAL_REISSUE_MS = 5000;

static const int MIN_ID = 1;
static const int MAX_ID = 6;
static const unsigned long AUTO_POS_INTERVAL_MS = 50;
static const unsigned long MOVE_COOLDOWN_MS = 50;
static const unsigned long POST_MOVE_REPORT_MS = 200;

// Una sola orden USB por punto. La secuencia se ejecuta de a un grupo
// verificado, sin superponer dos grupos en el bus de los servos.
static const uint8_t WAYPOINT_ARM_MOTORS = 5;
static const unsigned long WAYPOINT_SPACING_MS = 65;
static const unsigned long WAYPOINT_POLL_MS = 50;
static const unsigned long WAYPOINT_HEARTBEAT_MS = 750;
static const unsigned long WAYPOINT_GROUP_TIMEOUT_MS = 5000;
static const uint8_t WAYPOINT_QUIET_SAMPLES = 3;
static const float WAYPOINT_POSITION_TOLERANCE = 3.0f;
static const int WAYPOINT_QUIET_SPEED_RAW = 22; // ~2 grados/s

// Garra M6: 0 grados logicos = cerrada, 140 = abierta.
// ReadCurrent devuelve unidades crudas del registro del STS3215.
static const float GRIPPER_OPEN_DEGREES = 140.0f;
static const float GRIPPER_CLOSED_DEGREES = 0.0f;
static const float GRIPPER_STEP_DEGREES = 1.5f;
static const unsigned long GRIPPER_STEP_INTERVAL_MS = 40;
static const unsigned long GRIPPER_INRUSH_IGNORE_MS = 250;
static const unsigned long GRIPPER_CLOSE_TIMEOUT_MS = 15000;
static const float GRIPPER_CLOSED_TOLERANCE_DEGREES = 2.0f;
// 92 unidades * 6.5 mA/unidad = 598 mA, aproximadamente 600 mA.
static int gripperContactCurrentRaw = 92;
static const int GRIPPER_HARD_CURRENT_RAW = 200;
static const uint8_t GRIPPER_CONTACT_SAMPLES = 3;
// Vigilancia de corriente fuera del cierre: al abrir, al mover M6 por angulo y al
// sostener (docs/DIAGNOSTICO_GARRA.md). El cierre tiene la suya en
// processGripperClosing(). Con la fuente de 12 V / 5 A compartida, una garra
// apretando de mas puede cortar la fuente y dejar caer todo el brazo.
static const unsigned long GRIPPER_WATCH_INTERVAL_MS = 100;
static const unsigned long GRIPPER_MOVE_INRUSH_IGNORE_MS = 250;
static const unsigned long GRIPPER_MOVE_WATCH_MS = 9000;      // abrir desde cerrada tarda ~5,6 s
static const uint8_t GRIPPER_HARD_CURRENT_SAMPLES = 2;        // 200 ms seguidos sobre 200 (~1,3 A)
static const int GRIPPER_HOLD_LIMIT_RAW = 170;                // ~1,1 A sostenido al sostener
static const uint8_t GRIPPER_HOLD_LIMIT_SAMPLES = 20;         // 2 s seguidos
static const float GRIPPER_HOLD_RELIEF_DEGREES = 1.5f;        // aflojar de a 1,5 grados
static const uint8_t GRIPPER_HOLD_MAX_RELIEFS = 3;
static const uint8_t GRIPPER_READ_FAILURE_LIMIT = 3;
// Abrir agranda el contorno de la garra: se revisa el piso cada 4 grados de M6.
static const float GRIPPER_SWEEP_STEP_DEGREES = 4.0f;

// Cadena plana del modelo 3D calibrado (URDF + calibracion.json con los toques
// de mesa del 30/10/2026), en mm y grados. Debe coincidir con brazoia/planner.py;
// tests/test_planner.py compara ambos archivos.
static const float GEOM_H_BASE = 121.9f;
static const float GEOM_W_BASE = 35.0f;
static const float GEOM_L1 = 112.814f;
static const float GEOM_OFFSET_M3 = 27.0f;
static const float GEOM_L2 = 135.0f;
static const float GEOM_L3 = 60.1f;
static const float GEOM_M2_ZERO = -86.1243f;
static const float GEOM_M3_ZERO = -97.7163f;
static const float GEOM_M4_ZERO = -175.0883f;
// Rectangulo de la garra para autocolisiones, hasta la punta de los dedos.
static const float GEOM_GRIPPER_LEN = 106.4f;
static const float GEOM_GRIPPER_WIDTH = 65.0f;
// Unico limite de piso: el contorno de la garra a mas de 5 mm de la mesa.
// La franja de riesgo de 30 mm se elimino: con la cadena calibrada y el contorno
// segun M6 ya no hace falta.
static const float FLOOR_TOUCH_THRESHOLD = 5.0f;
static const float FLOOR_RISK_CLEARANCE = FLOOR_TOUCH_THRESHOLD;
// Si la pose actual ya esta bajo el margen (garra cerrada sobre el cubo cerca de
// la mesa), se aceptan poses que no queden mas cerca de la mesa, para poder salir.
static const float FLOOR_ESCAPE_TOLERANCE = 0.5f;

static const float OFFSET_SIGN = 1.0f;
static const bool MIRROR_X = true;

// Contorno de la garra alrededor del eje de M5, valido para cualquier giro de M5,
// segun la apertura de M6. t (mm) a lo largo de los dedos desde el eje de M5 y
// radio (mm) desde ese eje; sale de las mallas con 1 mm de margen.
struct EnvelopeSegment {
  float start;
  float end;
  float radius;
};
static const float ENVELOPE_CLOSED_MAX_M6 = 12.0f;
static const float ENVELOPE_HOLDING_MAX_M6 = 47.0f;
static const float ENVELOPE_HALF_OPEN_MAX_M6 = 128.0f;
static const EnvelopeSegment ENVELOPE_CLOSED[] = {
  {-1.0f, 60.0f, 40.0f}, {60.0f, 80.0f, 31.0f}, {80.0f, 95.0f, 31.0f}, {95.0f, 108.0f, 25.0f}
};
static const EnvelopeSegment ENVELOPE_HOLDING[] = {
  {-1.0f, 20.0f, 39.0f}, {20.0f, 40.0f, 44.0f}, {40.0f, 60.0f, 55.0f}, {60.0f, 80.0f, 65.0f},
  {80.0f, 95.0f, 68.0f}, {95.0f, 108.0f, 65.0f}
};
static const EnvelopeSegment ENVELOPE_HALF_OPEN[] = {
  {-15.0f, 0.0f, 101.0f}, {0.0f, 108.0f, 104.0f}
};
static const EnvelopeSegment ENVELOPE_OPEN[] = {
  {-26.0f, 0.0f, 98.0f}, {0.0f, 20.0f, 81.0f}, {20.0f, 40.0f, 49.0f}, {40.0f, 60.0f, 39.0f},
  {60.0f, 80.0f, 30.0f}, {80.0f, 95.0f, 23.0f}, {95.0f, 108.0f, 18.0f}
};
// Sin lectura de M6 vale el contorno de cualquier apertura.
static const EnvelopeSegment ENVELOPE_UNKNOWN[] = {
  {-26.0f, 0.0f, 101.0f}, {0.0f, 108.0f, 104.0f}
};

HardwareSerial ServoSerial(2);
SMS_STS servoBus;
String inputLine;
unsigned long lastAutoPosMs = 0;
unsigned long lastMoveCommandMs = 0;
bool pendingMoveReport = false;
uint8_t pendingMoveReportId = 0;
unsigned long pendingMoveReportAtMs = 0;
struct WaypointGroup {
  uint8_t size;
  uint8_t ids[2];
  float degrees[2];
};

struct Waypoint {
  uint32_t id;
  WaypointGroup groups[WAYPOINT_ARM_MOTORS];
  uint8_t size;
  uint8_t nextGroup;
  uint8_t quietSamples;
  uint8_t gripperReliefsAtStart;
  unsigned long heartbeatAtMs;
  unsigned long groupStartedAtMs;
  unsigned long lastPollAtMs;
};

Waypoint waypoint = {};
uint32_t lastWaypointId = 0;
bool discardInputLine = false;
bool gripperClosing = false;
float gripperCommandDegrees = GRIPPER_OPEN_DEGREES;
unsigned long gripperCloseStartedMs = 0;
unsigned long gripperLastStepMs = 0;
uint8_t gripperHighCurrentSamples = 0;
enum GripperWatchMode { WATCH_NONE, WATCH_MOVING, WATCH_HOLDING };
GripperWatchMode gripperWatch = WATCH_NONE;
unsigned long gripperWatchStartedMs = 0;
unsigned long gripperLastWatchMs = 0;
uint8_t gripperWatchHardSamples = 0;
uint8_t gripperWatchHighSamples = 0;
uint8_t gripperWatchReadFailures = 0;
uint8_t gripperReliefs = 0;
bool gripperTelemetry = false;
// Medicion de la fuente (12 V / 5 A compartida, que no da para mover todo a la vez):
// corriente de los seis servos y el voltaje mas bajo, cada POWER_SAMPLE_INTERVAL_MS.
bool powerTelemetry = false;
unsigned long lastPowerTelemMs = 0;

struct Point2 {
  float x;
  float y;
};

long degToPos(float deg) {
  return lround((deg / 360.0f) * 4095.0f);
}

float degToRad(float deg) {
  return deg * PI / 180.0f;
}

float posToDeg(int pos) {
  if (pos < 0) return -1.0f;
  return (pos * 360.0f) / 4095.0f;
}

bool servoPresent(uint8_t id) {
  return servoBus.Ping(id) != -1;
}

Point2 addPt(Point2 a, Point2 b) {
  Point2 r = {a.x + b.x, a.y + b.y};
  return r;
}

Point2 subPt(Point2 a, Point2 b) {
  Point2 r = {a.x - b.x, a.y - b.y};
  return r;
}

Point2 scalePt(Point2 v, float factor) {
  Point2 r = {v.x * factor, v.y * factor};
  return r;
}

float dotPt(Point2 a, Point2 b) {
  return a.x * b.x + a.y * b.y;
}

float lenPt(Point2 v) {
  return sqrtf(v.x * v.x + v.y * v.y);
}

Point2 vecPt(float length, float angleRad) {
  Point2 r = {length * cosf(angleRad), length * sinf(angleRad)};
  return r;
}

Point2 perpendicularOffset(float length, float angleRad) {
  Point2 r = {
    length * cosf(angleRad + OFFSET_SIGN * PI / 2.0f),
    length * sinf(angleRad + OFFSET_SIGN * PI / 2.0f)
  };
  return r;
}

float pointSegmentDistance(Point2 p, Point2 a, Point2 b) {
  Point2 ab = subPt(b, a);
  Point2 ap = subPt(p, a);
  float abLen2 = dotPt(ab, ab);
  if (abLen2 <= 1e-9f) {
    return lenPt(subPt(p, a));
  }
  float t = dotPt(ap, ab) / abLen2;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  Point2 proj = addPt(a, scalePt(ab, t));
  return lenPt(subPt(p, proj));
}

float orient(Point2 a, Point2 b, Point2 c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(Point2 a, Point2 b, Point2 p) {
  return (
    min(a.x, b.x) <= p.x && p.x <= max(a.x, b.x) &&
    min(a.y, b.y) <= p.y && p.y <= max(a.y, b.y)
  );
}

bool segmentsIntersect(Point2 a, Point2 b, Point2 c, Point2 d) {
  float o1 = orient(a, b, c);
  float o2 = orient(a, b, d);
  float o3 = orient(c, d, a);
  float o4 = orient(c, d, b);

  if (fabsf(o1) < 1e-6f && onSegment(a, b, c)) return true;
  if (fabsf(o2) < 1e-6f && onSegment(a, b, d)) return true;
  if (fabsf(o3) < 1e-6f && onSegment(c, d, a)) return true;
  if (fabsf(o4) < 1e-6f && onSegment(c, d, b)) return true;
  return ((o1 > 0) != (o2 > 0)) && ((o3 > 0) != (o4 > 0));
}

bool pointInRect(Point2 p, float xMin, float xMax, float yMin, float yMax) {
  return xMin <= p.x && p.x <= xMax && yMin <= p.y && p.y <= yMax;
}

void computePosePoints(float m2Deg, float m3Deg, float m4Deg, Point2 &p0, Point2 &p1, Point2 &p1o, Point2 &p2, Point2 &p3) {
  float a1 = degToRad(m2Deg + GEOM_M2_ZERO);
  float a2 = degToRad(m3Deg + GEOM_M3_ZERO);
  float a3 = degToRad(m4Deg + GEOM_M4_ZERO);

  p0 = {0.0f, GEOM_H_BASE};
  p1 = addPt(p0, vecPt(GEOM_L1, a1));
  p1o = addPt(p1, perpendicularOffset(GEOM_OFFSET_M3, a1));
  p2 = addPt(p1o, vecPt(GEOM_L2, a1 + a2));
  p3 = addPt(p2, vecPt(GEOM_L3, a1 + a2 + a3));

  if (MIRROR_X) {
    p0.x = -p0.x;
    p1.x = -p1.x;
    p1o.x = -p1o.x;
    p2.x = -p2.x;
    p3.x = -p3.x;
  }
}

void computeGripperCorners(float m2Deg, float m3Deg, float m4Deg, Point2 corners[4], Point2 &p0, Point2 &p1, Point2 &p1o, Point2 &p2, Point2 &p3) {
  computePosePoints(m2Deg, m3Deg, m4Deg, p0, p1, p1o, p2, p3);
  Point2 dir = subPt(p3, p2);
  float dirLen = lenPt(dir);
  Point2 u = {1.0f, 0.0f};
  if (dirLen > 1e-6f) {
    u = {dir.x / dirLen, dir.y / dirLen};
  }
  Point2 n = {-u.y, u.x};
  float halfW = GEOM_GRIPPER_WIDTH / 2.0f;

  corners[0] = addPt(p3, scalePt(n, -halfW));
  corners[1] = addPt(p3, scalePt(n, halfW));
  corners[2] = addPt(corners[1], scalePt(u, GEOM_GRIPPER_LEN));
  corners[3] = addPt(corners[0], scalePt(u, GEOM_GRIPPER_LEN));
}

void selectGripperEnvelope(float m6Deg, const EnvelopeSegment *&segments, int &count) {
  if (isnan(m6Deg)) {
    segments = ENVELOPE_UNKNOWN;
    count = sizeof(ENVELOPE_UNKNOWN) / sizeof(ENVELOPE_UNKNOWN[0]);
  } else if (m6Deg <= ENVELOPE_CLOSED_MAX_M6) {
    segments = ENVELOPE_CLOSED;
    count = sizeof(ENVELOPE_CLOSED) / sizeof(ENVELOPE_CLOSED[0]);
  } else if (m6Deg <= ENVELOPE_HOLDING_MAX_M6) {
    segments = ENVELOPE_HOLDING;
    count = sizeof(ENVELOPE_HOLDING) / sizeof(ENVELOPE_HOLDING[0]);
  } else if (m6Deg <= ENVELOPE_HALF_OPEN_MAX_M6) {
    segments = ENVELOPE_HALF_OPEN;
    count = sizeof(ENVELOPE_HALF_OPEN) / sizeof(ENVELOPE_HALF_OPEN[0]);
  } else {
    segments = ENVELOPE_OPEN;
    count = sizeof(ENVELOPE_OPEN) / sizeof(ENVELOPE_OPEN[0]);
  }
}

// Altura minima (mm) del contorno de la garra sobre la mesa, con cualquier giro de M5.
float gripperFloorClearance(float m2Deg, float m3Deg, float m4Deg, float m6Deg) {
  Point2 p0, p1, p1o, p2, p3;
  computePosePoints(m2Deg, m3Deg, m4Deg, p0, p1, p1o, p2, p3);
  Point2 dir = subPt(p3, p2);
  float dirLen = lenPt(dir);
  Point2 u = {1.0f, 0.0f};
  if (dirLen > 1e-6f) {
    u = {dir.x / dirLen, dir.y / dirLen};
  }
  Point2 n = {-u.y, u.x};
  const EnvelopeSegment *segments = nullptr;
  int count = 0;
  selectGripperEnvelope(m6Deg, segments, count);
  float lowest = 1e9f;
  for (int i = 0; i < count; i++) {
    float along[2] = {segments[i].start, segments[i].end};
    for (int a = 0; a < 2; a++) {
      for (int side = -1; side <= 1; side += 2) {
        float y = p3.y + along[a] * u.y + side * segments[i].radius * n.y;
        if (y < lowest) lowest = y;
      }
    }
  }
  return lowest;
}

// Referencia de escape: la altura actual si ya esta bajo el margen; si no, NAN.
float floorEscapeReference(float m2Deg, float m3Deg, float m4Deg, float m6Deg) {
  float clearance = gripperFloorClearance(m2Deg, m3Deg, m4Deg, m6Deg);
  return clearance <= FLOOR_TOUCH_THRESHOLD ? clearance : NAN;
}

bool floorBlocked(float clearance, float threshold, float escapeFrom) {
  if (clearance > threshold) return false;
  return isnan(escapeFrom) || clearance < escapeFrom - FLOOR_ESCAPE_TOLERANCE;
}

// Abrir de m6 (NAN: sin lectura, se revisa toda la apertura) a GRIPPER_OPEN_DEGREES
// en la pose actual. Igual que planner.gripper_sweep_problem(): cerrar no se revisa,
// es el agarre mismo cerca de la mesa.
bool gripperOpeningBlocked(float m2, float m3, float m4, float m6, float &worst, float &worstM6) {
  float low = isnan(m6) ? 0.0f : m6;
  float high = 140.0f;
  worst = 1e9f;
  worstM6 = low;
  if (!isnan(m6) && high <= m6) return false;
  int steps = max(1, (int)ceilf((high - low) / GRIPPER_SWEEP_STEP_DEGREES));
  for (int i = 0; i <= steps; i++) {
    float angle = low + (high - low) * (float)i / (float)steps;
    float clearance = gripperFloorClearance(m2, m3, m4, angle);
    if (clearance < worst) {
      worst = clearance;
      worstM6 = angle;
    }
  }
  // Sin lectura de M6 no hay pose de partida conocida: no se da margen de escape.
  float escapeFrom = isnan(m6) ? NAN : floorEscapeReference(m2, m3, m4, m6);
  return floorBlocked(worst, FLOOR_TOUCH_THRESHOLD, escapeFrom);
}

// M6 actual, o NAN si no se puede leer (se usa el contorno mas desfavorable).
float currentGripperDegreesOrNan() {
  float m6 = NAN;
  if (!getCurrentLogicalDegrees(6, m6)) {
    return NAN;
  }
  return m6;
}

bool collidesWithBase(Point2 corners[4]) {
  float xMin = -GEOM_W_BASE / 2.0f;
  float xMax = GEOM_W_BASE / 2.0f;
  float yMin = 0.0f;
  float yMax = GEOM_H_BASE;
  for (int i = 0; i < 4; i++) {
    if (pointInRect(corners[i], xMin, xMax, yMin, yMax)) return true;
  }

  Point2 rectEdgesA[4] = {
    {xMin, yMin}, {xMax, yMin}, {xMax, yMax}, {xMin, yMax}
  };
  Point2 rectEdgesB[4] = {
    {xMax, yMin}, {xMax, yMax}, {xMin, yMax}, {xMin, yMin}
  };

  for (int i = 0; i < 4; i++) {
    Point2 a = corners[i];
    Point2 b = corners[(i + 1) % 4];
    for (int j = 0; j < 4; j++) {
      if (segmentsIntersect(a, b, rectEdgesA[j], rectEdgesB[j])) return true;
    }
  }
  return false;
}

bool collidesWithLink(Point2 corners[4], Point2 a, Point2 b, float thickness) {
  float radius = thickness / 2.0f;
  for (int i = 0; i < 4; i++) {
    if (pointSegmentDistance(corners[i], a, b) <= radius) return true;
  }
  for (int i = 0; i < 4; i++) {
    Point2 e0 = corners[i];
    Point2 e1 = corners[(i + 1) % 4];
    if (segmentsIntersect(e0, e1, a, b)) return true;
    if (pointSegmentDistance(e0, a, b) <= radius) return true;
    if (pointSegmentDistance(e1, a, b) <= radius) return true;
  }
  return false;
}

bool classifyPoseInvalid(
  float m2Deg,
  float m3Deg,
  float m4Deg,
  float m6Deg,
  String &reason,
  bool allowFloorRisk = false,
  float escapeFrom = NAN
) {
  Point2 corners[4];
  Point2 p0, p1, p1o, p2, p3;
  computeGripperCorners(m2Deg, m3Deg, m4Deg, corners, p0, p1, p1o, p2, p3);

  float clearance = gripperFloorClearance(m2Deg, m3Deg, m4Deg, m6Deg);
  if (floorBlocked(clearance, FLOOR_TOUCH_THRESHOLD, escapeFrom)) {
    reason = "PISO";
    return true;
  }
  if (collidesWithBase(corners)) {
    reason = "COLISION_BASE";
    return true;
  }
  if (collidesWithLink(corners, p0, p1, 25.0f)) {
    reason = "COLISION_L1";
    return true;
  }
  if (collidesWithLink(corners, p1o, p2, 25.0f)) {
    reason = "COLISION_L2";
    return true;
  }
  if (!allowFloorRisk && floorBlocked(clearance, FLOOR_RISK_CLEARANCE, escapeFrom)) {
    reason = "RIESGO_PISO";
    return true;
  }

  reason = "OK";
  return false;
}

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

void printHelp() {
  printDivider();
  Serial.println(F("Comandos disponibles:"));
  Serial.println(F("  <id> <grados>"));
  Serial.println(F("  pair <id1> <grados1> <id2> <grados2>"));
  Serial.println(F("  sync <ms> <id> <grados> [<id> <grados> ...]  (M1..M5 juntos, ms=0: maxima)"));
  Serial.println(F("  caps"));
  Serial.println(F("  speed max 450 [45]   (pasos/s y aceleracion de M1..M5; techo 1000)"));
  Serial.println(F("  power guard on|off"));
  Serial.println(F("  power guard 95 80 3000 4500  (lento/corte en decimas de V y en mA)"));
  Serial.println(F("  read <id>"));
  Serial.println(F("  scan"));
  Serial.println(F("  posall"));
  Serial.println(F("  arm torque off  (solamente M1..M5)"));
  Serial.println(F("  arm torque on   (solamente M1..M5)"));
  Serial.println(F("  torque off      (emergencia: M1..M6)"));
  Serial.println(F("  torque on       (todos: M1..M6)"));
  Serial.println(F("  gripper open"));
  Serial.println(F("  gripper close"));
  Serial.println(F("  waypoint ID N MOTOR GRADOS ... [N MOTOR GRADOS ...]"));
  Serial.println(F("  waypoint keepalive ID | waypoint cancel ID"));
  Serial.println(F("  gripper stop"));
  Serial.println(F("  gripper release  (torque OFF solamente en M6)"));
  Serial.println(F("  gripper current 92  (aprox. 600 mA)"));
  Serial.println(F("  gripper telem on|off  (corriente de M6 cada 0.1 s al abrir y sostener)"));
  Serial.println(F("  power telem on|off    (corriente de M1..M6 y voltaje minimo cada 0.1 s)"));
  Serial.println(F("  motor2 90"));
  Serial.println(F("  pair 1 90 2 80"));
  Serial.println(F("  help"));
  Serial.print(F("Cooldown entre movimientos: "));
  Serial.print(MOVE_COOLDOWN_MS);
  Serial.println(F(" ms"));
  Serial.println(F("Limites configurados:"));
  Serial.println(F("  ID 1:    60..300"));
  Serial.println(F("  ID 2:    90..270"));
  Serial.println(F("  ID 3:    90..276"));
  Serial.println(F("  ID 4:    75..278 con reglas dinamicas"));
  Serial.println(F("  ID 5:    0..340"));
  Serial.println(F("  ID 6:    0..140 logico -> 10..150 real"));
  Serial.println(F("Reglas dinamicas de test para M4:"));
  Serial.println(F("  Si M2<=100 y M3<=110 -> M4<=130"));
  Serial.println(F("  Si M2<=120 y M3<=100 -> M4<=120"));
  Serial.println(F("  Si M2>=240 y M3>=140 -> M4<=200"));
  Serial.println(F("  Si M2>=260 -> M4<=185"));
  Serial.println(F("  Si M3>=250 -> M4<=230"));
  Serial.println(F("  Si M2>=185 y M3>=245 -> M4<=250"));
  Serial.println(F("  Si M2>=200 y M3>=255 -> M4<=245"));
  Serial.println(F("Ejemplos:"));
  Serial.println(F("  1 180"));
  Serial.println(F("  2 90"));
  Serial.println(F("  4 150"));
  Serial.println(F("  read 3"));
  Serial.println(F("  posall"));
  printDivider();
}

bool getUserAngleLimits(uint8_t id, float &minDeg, float &maxDeg) {
  switch (id) {
    case 1:
      minDeg = 60.0f;
      maxDeg = 300.0f;
      return true;
    case 2:
      minDeg = 90.0f;
      maxDeg = 270.0f;
      return true;
    case 3:
      minDeg = 90.0f;
      maxDeg = 276.0f;
      return true;
    case 4:
      minDeg = 75.0f;
      maxDeg = 278.0f;
      return true;
    case 5:
      minDeg = 0.0f;
      maxDeg = 340.0f;
      return true;
    case 6:
      minDeg = 0.0f;
      maxDeg = 140.0f;
      return true;
    default:
      return false;
  }
}

float toRealServoDegrees(uint8_t id, float userDegrees) {
  if (id == 6) {
    return userDegrees + 10.0f;
  }
  return userDegrees;
}

bool getCurrentLogicalDegrees(uint8_t id, float &degreesOut) {
  if (!servoPresent(id)) {
    return false;
  }

  int pos = servoBus.ReadPos(id);
  if (pos < 0) {
    return false;
  }

  float deg = posToDeg(pos);
  if (id == 6) {
    deg -= 10.0f;
  }
  degreesOut = deg;
  return true;
}

// Velocidad de un motor del brazo con la escala del supervisor de la fuente.
u16 scaledArmSpeed(float base) {
  float value = base * speedScalePct / 100.0f;
  return (u16)max((float)SYNC_MIN_SPEED, value);
}

void rememberArmGoal(uint8_t id, int position, float baseSpeed) {
  if (id < 1 || id > 5) return;
  armGoal[id] = position;
  armBaseSpeed[id] = (u16)baseSpeed;
  armGoalValid[id] = true;
  armGoalAtMs[id] = millis();
}

// Con el torque cambiado (o el brazo movido a mano) un destino viejo ya no vale.
void forgetArmGoals() {
  for (uint8_t id = 1; id <= 5; id++) armGoalValid[id] = false;
}

bool getCurrentPoseDegrees(float &m2, float &m3, float &m4) {
  return getCurrentLogicalDegrees(2, m2) &&
         getCurrentLogicalDegrees(3, m3) &&
         getCurrentLogicalDegrees(4, m4);
}

float getDynamicM4Max(float m2, float m3) {
  float maxM4 = 278.0f;

  if (m2 <= 100.0f && m3 <= 110.0f) {
    maxM4 = min(maxM4, 130.0f);
  }
  if (m2 <= 120.0f && m3 <= 100.0f) {
    maxM4 = min(maxM4, 120.0f);
  }
  if (m2 >= 240.0f && m3 >= 140.0f) {
    maxM4 = min(maxM4, 200.0f);
  }
  if (m2 >= 260.0f) {
    maxM4 = min(maxM4, 185.0f);
  }
  if (m3 >= 250.0f) {
    maxM4 = min(maxM4, 230.0f);
  }
  if (m2 >= 185.0f && m3 >= 245.0f) {
    maxM4 = min(maxM4, 250.0f);
  }
  if (m2 >= 200.0f && m3 >= 255.0f) {
    maxM4 = min(maxM4, 245.0f);
  }

  return maxM4;
}

bool poseAllowedForMove(
  uint8_t id,
  float candidateDegrees,
  float currentM2,
  float currentM3,
  float currentM4,
  float currentM6,
  float escapeFrom,
  bool allowFloorRisk = false
) {
  float targetM2 = currentM2;
  float targetM3 = currentM3;
  float targetM4 = currentM4;
  if (id == 2) targetM2 = candidateDegrees;
  if (id == 3) targetM3 = candidateDegrees;
  if (id == 4) targetM4 = candidateDegrees;

  if (id == 4 && candidateDegrees > getDynamicM4Max(currentM2, currentM3)) {
    return false;
  }

  if (id == 2 || id == 3 || id == 4) {
    String reason;
    return !classifyPoseInvalid(
      targetM2, targetM3, targetM4, currentM6, reason, allowFloorRisk, escapeFrom
    );
  }
  return true;
}

bool validateDynamicRules(
  uint8_t id,
  float &requestedDegrees,
  bool allowFloorRisk = false
) {
  float m2 = 0.0f;
  float m3 = 0.0f;
  float m4 = 0.0f;
  if (!getCurrentPoseDegrees(m2, m3, m4)) {
    Serial.println(F("No se pudo leer M2/M3/M4 para validar la pose."));
    return false;
  }
  float m6 = currentGripperDegreesOrNan();
  float escapeFrom = floorEscapeReference(m2, m3, m4, m6);

  if (poseAllowedForMove(
        id, requestedDegrees, m2, m3, m4, m6, escapeFrom, allowFloorRisk
      )) {
    return true;
  }

  float minDeg = 0.0f;
  float maxDeg = 0.0f;
  if (!getUserAngleLimits(id, minDeg, maxDeg)) {
    return false;
  }

  const float SEARCH_STEP_DEGREES = 0.5f;
  float original = requestedDegrees;
  int maxSteps = (int)ceilf((maxDeg - minDeg) / SEARCH_STEP_DEGREES);

  for (int step = 1; step <= maxSteps; step++) {
    float delta = step * SEARCH_STEP_DEGREES;
    float lower = original - delta;
    float upper = original + delta;

    if (lower >= minDeg && poseAllowedForMove(
          id, lower, m2, m3, m4, m6, escapeFrom, allowFloorRisk
        )) {
      requestedDegrees = lower;
      Serial.print(F("AJUSTE SEGURO ID "));
      Serial.print(id);
      Serial.print(F(": "));
      Serial.print(original, 2);
      Serial.print(F(" -> "));
      Serial.println(requestedDegrees, 2);
      return true;
    }
    if (upper <= maxDeg && poseAllowedForMove(
          id, upper, m2, m3, m4, m6, escapeFrom, allowFloorRisk
        )) {
      requestedDegrees = upper;
      Serial.print(F("AJUSTE SEGURO ID "));
      Serial.print(id);
      Serial.print(F(": "));
      Serial.print(original, 2);
      Serial.print(F(" -> "));
      Serial.println(requestedDegrees, 2);
      return true;
    }
  }

  Serial.print(F("Bloqueado: no existe correccion segura cercana para ID "));
  Serial.println(id);
  return false;
}

bool validatePairTrajectory(
  uint8_t id1,
  float degrees1,
  uint8_t id2,
  float degrees2,
  String &reason,
  bool allowFloorRisk = false,
  bool requireKnownGripper = false
) {
  float currentM2 = 0.0f;
  float currentM3 = 0.0f;
  float currentM4 = 0.0f;
  if (!getCurrentPoseDegrees(currentM2, currentM3, currentM4)) {
    reason = "LECTURA_M2_M3_M4";
    return false;
  }

  float targetM2 = currentM2;
  float targetM3 = currentM3;
  float targetM4 = currentM4;
  float currentM6 = currentGripperDegreesOrNan();
  if (requireKnownGripper && isnan(currentM6)) {
    reason = "LECTURA_M6";
    return false;
  }
  float targetM6 = currentM6;
  if (id1 == 2) targetM2 = degrees1;
  if (id1 == 3) targetM3 = degrees1;
  if (id1 == 4) targetM4 = degrees1;
  if (id1 == 6) targetM6 = degrees1;
  if (id2 == 2) targetM2 = degrees2;
  if (id2 == 3) targetM3 = degrees2;
  if (id2 == 4) targetM4 = degrees2;
  if (id2 == 6) targetM6 = degrees2;
  float escapeFrom = floorEscapeReference(currentM2, currentM3, currentM4, currentM6);

  // Revisar puntos intermedios, no solamente la pose final. Esto evita que
  // dos articulaciones crucen juntas una region de piso o autocolision.
  const int INTERPOLATION_STEPS = 20;
  for (int step = 1; step <= INTERPOLATION_STEPS; step++) {
    float t = (float)step / (float)INTERPOLATION_STEPS;
    float m2 = currentM2 + (targetM2 - currentM2) * t;
    float m3 = currentM3 + (targetM3 - currentM3) * t;
    float m4 = currentM4 + (targetM4 - currentM4) * t;
    float m6 = currentM6 + (targetM6 - currentM6) * t;

    if (m4 > getDynamicM4Max(m2, m3)) {
      reason = "REGLA_DINAMICA_M4";
      return false;
    }
    if (classifyPoseInvalid(m2, m3, m4, m6, reason, allowFloorRisk, escapeFrom)) {
      return false;
    }
  }

  reason = "OK";
  return true;
}

void readOneServo(uint8_t id) {
  if (!servoPresent(id)) {
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": sin respuesta"));
    return;
  }

  int pos = servoBus.ReadPos(id);
  int voltage = servoBus.ReadVoltage(id);
  int temp = servoBus.ReadTemper(id);

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": pos="));
  Serial.print(pos);
  Serial.print(F(" (~"));
  Serial.print(posToDeg(pos), 1);
  Serial.print(F(" deg) | V="));
  Serial.print(voltage);
  Serial.print(F(" | T="));
  Serial.println(temp);
}

void scanAll() {
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    readOneServo(id);
  }
  printDivider();
}

void printAllPositions() {
  bool first = true;
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (!first) {
      Serial.print(F(" | "));
    }
    first = false;

    Serial.print(F("M"));
    Serial.print(id);
    Serial.print(F("="));

    if (!servoPresent(id)) {
      Serial.print(F("NA"));
      continue;
    }

    int pos = servoBus.ReadPos(id);
    Serial.print(pos);
    Serial.print(F("("));
    Serial.print(posToDeg(pos), 1);
    Serial.print(F("deg)"));
  }
  Serial.println();
  printDivider();
}

void autoPrintPositionsIfNeeded() {
  unsigned long now = millis();
  if (now - lastAutoPosMs < AUTO_POS_INTERVAL_MS) {
    return;
  }

  lastAutoPosMs = now;

  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (!servoPresent(id)) {
      continue;
    }

    int pos = servoBus.ReadPos(id);
    Serial.print(F("POS ID "));
    Serial.print(id);
    Serial.print(F(": "));
    Serial.print(pos);
    Serial.print(F(" (~"));
    Serial.print(posToDeg(pos), 1);
    Serial.println(F(" deg)"));
  }
}

void processPendingMoveReport() {
  if (!pendingMoveReport) {
    return;
  }

  if (millis() < pendingMoveReportAtMs) {
    return;
  }

  pendingMoveReport = false;
  readOneServo(pendingMoveReportId);
  printDivider();
}

void moveServoToDegrees(
  uint8_t id,
  float degrees,
  bool allowFloorRisk = false
) {
  float minDeg = 0.0f;
  float maxDeg = 0.0f;
  unsigned long now = millis();

  if (id < MIN_ID || id > MAX_ID) {
    Serial.println(F("ID invalido. Usa 1..6."));
    return;
  }

  if (id == 6) {
    if (gripperClosing) {
      Serial.println(F("GRIPPER_STOP reason=manual_position_command detail=se_recibio_un_angulo_para_M6"));
    }
    gripperClosing = false;
    gripperHighCurrentSamples = 0;
    startGripperWatch(WATCH_MOVING);
  }

  if (now - lastMoveCommandMs < MOVE_COOLDOWN_MS) {
    Serial.print(F("Cooldown activo. Espera "));
    Serial.print(MOVE_COOLDOWN_MS - (now - lastMoveCommandMs));
    Serial.println(F(" ms antes del siguiente movimiento."));
    return;
  }

  if (!getUserAngleLimits(id, minDeg, maxDeg)) {
    Serial.println(F("No hay limites definidos para ese ID."));
    return;
  }

  if (degrees < minDeg || degrees > maxDeg) {
    float original = degrees;
    degrees = constrain(degrees, minDeg, maxDeg);
    Serial.print(F("AJUSTE LIMITE ID "));
    Serial.print(id);
    Serial.print(F(": "));
    Serial.print(original, 2);
    Serial.print(F(" -> "));
    Serial.println(degrees, 2);
  }

  if (!servoPresent(id)) {
    Serial.println(F("Ese servo no responde."));
    return;
  }

  if (!validateDynamicRules(id, degrees, allowFloorRisk)) {
    printDivider();
    return;
  }

  if (allowFloorRisk) {
    Serial.println(F("GRABARB_OVERRIDE RIESGO_PISO permitido; PISO y colisiones siguen bloqueados"));
  }

  float realDegrees = toRealServoDegrees(id, degrees);
  long targetPos = degToPos(realDegrees);

  Serial.print(F("Moviendo ID "));
  Serial.print(id);
  Serial.print(F(" a "));
  Serial.print(degrees, 1);
  Serial.print(F(" deg"));
  if (id == 6) {
    Serial.print(F(" (real "));
    Serial.print(realDegrees, 1);
    Serial.print(F(" deg)"));
  }
  Serial.print(F(" -> pos "));
  Serial.println(targetPos);

  lastMoveCommandMs = now;
  if (id <= 5) {
    servoBus.WritePosEx(id, (int)targetPos, scaledArmSpeed(armMaxSpeed), armAcc);
    rememberArmGoal(id, targetPos, armMaxSpeed);
  } else {
    servoBus.WritePosEx(id, (int)targetPos, MOVE_SPEED, MOVE_ACC);
  }
  pendingMoveReport = true;
  pendingMoveReportId = id;
  pendingMoveReportAtMs = now + POST_MOVE_REPORT_MS;
}

void writeSynchronizedPair(
  uint8_t id1, float degrees1, float current1,
  uint8_t id2, float degrees2, float current2, u16 speeds[2]
) {
  float delta1 = fabsf(degrees1 - current1);
  float delta2 = fabsf(degrees2 - current2);
  float maxDelta = max(delta1, delta2);

  u8 ids[2] = {id1, id2};
  s16 positions[2] = {
    (s16)degToPos(toRealServoDegrees(id1, degrees1)),
    (s16)degToPos(toRealServoDegrees(id2, degrees2))
  };
  // Un par del brazo usa `speed max`; con M6, la velocidad de siempre. La escala del
  // supervisor se aplica a los dos por igual para que sigan llegando juntos.
  bool arm = id1 <= 5 && id2 <= 5;
  float cruise = arm ? armMaxSpeed : MOVE_SPEED;
  float base[2] = {cruise, cruise};
  if (maxDelta > 0.1f) {
    base[0] = max(80.0f, cruise * delta1 / maxDelta);
    base[1] = max(80.0f, cruise * delta2 / maxDelta);
  }
  speeds[0] = scaledArmSpeed(base[0]);
  speeds[1] = scaledArmSpeed(base[1]);
  u8 accelerations[2] = {arm ? armAcc : MOVE_ACC, arm ? armAcc : MOVE_ACC};

  servoBus.SyncWritePosEx(ids, 2, positions, speeds, accelerations);
  rememberArmGoal(id1, positions[0], base[0]);
  rememberArmGoal(id2, positions[1], base[1]);
}

void moveTwoServos(
  uint8_t id1,
  float degrees1,
  uint8_t id2,
  float degrees2,
  bool allowFloorRisk = false
) {
  unsigned long now = millis();
  if (id1 < MIN_ID || id1 > MAX_ID || id2 < MIN_ID || id2 > MAX_ID) {
    Serial.println(F("PAIR_REJECTED reason=ID_invalido usa_1_a_6"));
    return;
  }
  if (id1 == id2) {
    Serial.println(F("PAIR_REJECTED reason=IDs_repetidos"));
    return;
  }

  float min1, max1, min2, max2;
  getUserAngleLimits(id1, min1, max1);
  getUserAngleLimits(id2, min2, max2);
  if (degrees1 < min1 || degrees1 > max1) {
    Serial.print(F("PAIR_REJECTED reason=fuera_de_rango ID="));
    Serial.println(id1);
    return;
  }
  if (degrees2 < min2 || degrees2 > max2) {
    Serial.print(F("PAIR_REJECTED reason=fuera_de_rango ID="));
    Serial.println(id2);
    return;
  }
  if (now - lastMoveCommandMs < MOVE_COOLDOWN_MS) {
    Serial.println(F("PAIR_REJECTED reason=cooldown"));
    return;
  }
  if (!servoPresent(id1) || !servoPresent(id2)) {
    Serial.println(F("PAIR_REJECTED reason=servo_sin_respuesta"));
    return;
  }

  String safetyReason;
  if (!validatePairTrajectory(
        id1, degrees1, id2, degrees2, safetyReason, allowFloorRisk
      )) {
    Serial.print(F("PAIR_REJECTED reason=trayectoria_insegura detail="));
    Serial.println(safetyReason);
    return;
  }

  if ((id1 == 6 || id2 == 6) && gripperClosing) {
    Serial.println(F("GRIPPER_STOP reason=manual_pair_command detail=se_recibio_un_angulo_para_M6"));
    gripperClosing = false;
    gripperHighCurrentSamples = 0;
  }
  if (id1 == 6 || id2 == 6) {
    startGripperWatch(WATCH_MOVING);
  }

  float current1 = degrees1;
  float current2 = degrees2;
  getCurrentLogicalDegrees(id1, current1);
  getCurrentLogicalDegrees(id2, current2);
  u16 speeds[2];
  writeSynchronizedPair(id1, degrees1, current1, id2, degrees2, current2, speeds);
  lastMoveCommandMs = now;
  pendingMoveReport = false;

  if (allowFloorRisk) {
    Serial.println(F("GRABARB_OVERRIDE RIESGO_PISO permitido; PISO y colisiones siguen bloqueados"));
  }

  Serial.print(F("PAIR_MOVE_OK ID1="));
  Serial.print(id1);
  Serial.print(F(" target1="));
  Serial.print(degrees1, 2);
  Serial.print(F(" speed1="));
  Serial.print(speeds[0]);
  Serial.print(F(" ID2="));
  Serial.print(id2);
  Serial.print(F(" target2="));
  Serial.print(degrees2, 2);
  Serial.print(F(" speed2="));
  Serial.println(speeds[1]);
}

const __FlashStringHelper *gripperWatchName() {
  if (gripperWatch == WATCH_MOVING) return F("moving");
  if (gripperWatch == WATCH_HOLDING) return F("holding");
  return F("none");
}

void startGripperWatch(GripperWatchMode mode) {
  gripperWatch = mode;
  gripperWatchStartedMs = millis();
  gripperLastWatchMs = 0;
  gripperWatchHardSamples = 0;
  gripperWatchHighSamples = 0;
  gripperWatchReadFailures = 0;
  gripperReliefs = 0;
}

void freezeGripperAtCurrentPosition(const char *reason, int currentRaw) {
  int position = servoBus.ReadPos(6);
  if (position >= 0) {
    servoBus.WritePosEx(6, position, 120, 15);
    gripperCommandDegrees = posToDeg(position) - 10.0f;
  }
  gripperClosing = false;
  gripperHighCurrentSamples = 0;
  Serial.print(F("GRIPPER_HOLD reason="));
  Serial.print(reason);
  Serial.print(F(" current_raw="));
  Serial.print(currentRaw);
  Serial.print(F(" current_ma_aprox="));
  Serial.print(currentRaw * 6.5f, 1);
  Serial.print(F(" position_raw="));
  Serial.print(position);
  Serial.print(F(" logical_deg="));
  Serial.println(gripperCommandDegrees, 1);
  startGripperWatch(WATCH_HOLDING);
}

void openGripper() {
  float m2 = 0.0f;
  float m3 = 0.0f;
  float m4 = 0.0f;
  if (!getCurrentPoseDegrees(m2, m3, m4)) {
    Serial.println(F("GRIPPER_ERROR reason=sin_pose detail=no_se_pudo_leer_M2_M4_para_revisar_el_piso"));
    return;
  }
  float worst = 0.0f;
  float worstM6 = 0.0f;
  if (gripperOpeningBlocked(m2, m3, m4, currentGripperDegreesOrNan(), worst, worstM6)) {
    Serial.print(F("GRIPPER_ERROR reason=floor detail=abrir_la_garra_barre_la_mesa clearance_mm="));
    Serial.print(worst, 1);
    Serial.print(F(" m6_deg="));
    Serial.println(worstM6, 1);
    return;
  }
  if (gripperClosing) {
    Serial.println(F("GRIPPER_STOP reason=open_command detail=se_cancelo_el_cierre_para_abrir"));
  }
  gripperClosing = false;
  gripperHighCurrentSamples = 0;
  servoBus.EnableTorque(6, 1);
  float realDegrees = toRealServoDegrees(6, GRIPPER_OPEN_DEGREES);
  servoBus.WritePosEx(6, (int)degToPos(realDegrees), 300, 25);
  gripperCommandDegrees = GRIPPER_OPEN_DEGREES;
  Serial.println(F("GRIPPER_OPEN_OK target=140.0"));
  startGripperWatch(WATCH_MOVING);
}

void startClosingGripper() {
  if (gripperClosing) {
    // Una segunda pulsacion no reinicia cronometro, contador ni exclusion inicial.
    Serial.println(F("GRIPPER_CLOSE_IGNORED reason=ya_esta_cerrando"));
    return;
  }
  if (!servoPresent(6)) {
    Serial.println(F("ID 6: sin respuesta"));
    return;
  }
  int position = servoBus.ReadPos(6);
  if (position < 0) {
    Serial.println(F("No se pudo leer la posicion de la garra."));
    return;
  }
  servoBus.EnableTorque(6, 1);
  gripperCommandDegrees = constrain(posToDeg(position) - 10.0f, GRIPPER_CLOSED_DEGREES, GRIPPER_OPEN_DEGREES);
  gripperWatch = WATCH_NONE;  // el cierre se vigila en processGripperClosing()
  gripperClosing = true;
  gripperCloseStartedMs = millis();
  gripperLastStepMs = 0;
  gripperHighCurrentSamples = 0;
  Serial.print(F("GRIPPER_CLOSE_START from="));
  Serial.print(gripperCommandDegrees, 1);
  Serial.print(F(" contact_raw="));
  Serial.println(gripperContactCurrentRaw);
}

void stopGripper() {
  int currentRaw = abs(servoBus.ReadCurrent(6));
  freezeGripperAtCurrentPosition("manual", currentRaw);
}

void releaseGripperTorque() {
  if (gripperClosing) {
    Serial.println(F("GRIPPER_STOP reason=torque_release detail=se_cancelo_el_cierre"));
  }
  gripperClosing = false;
  gripperHighCurrentSamples = 0;
  gripperWatch = WATCH_NONE;
  int result = servoBus.EnableTorque(6, 0);
  Serial.print(F("GRIPPER_TORQUE_OFF reason=manual_release result="));
  Serial.println(result);
}

// Sin lectura confiable no se puede vigilar: se intenta quitar el torque de M6 y se
// informa si el servo no lo confirma.
void releaseGripperAfterFailure(const char *reason) {
  gripperWatch = WATCH_NONE;
  int result = servoBus.EnableTorque(6, 0);
  Serial.print(F("GRIPPER_TORQUE_OFF reason="));
  Serial.print(reason);
  Serial.print(F(" result="));
  Serial.println(result);
  if (result != 1) {
    Serial.println(F("GRIPPER_ERROR reason=torque_off_sin_confirmar detail=desconecta_la_fuente_si_la_garra_sigue_apretando"));
  }
}

// Primero el corte, sin otras operaciones de bus antes, y despues el aviso con el
// resultado: si el servo no lo confirma, la app lo muestra como falla.
void torqueOffForOvercurrent() {
  gripperWatch = WATCH_NONE;
  int result = servoBus.EnableTorque(6, 0);
  Serial.print(F("GRIPPER_TORQUE_OFF proteccion de sobrecorriente result="));
  Serial.println(result);
  if (result != 1) {
    Serial.println(F("GRIPPER_ERROR reason=torque_off_sin_confirmar detail=desconecta_la_fuente_si_la_garra_sigue_apretando"));
  }
}

void stopWatchWithTorqueOff(int currentRaw, const __FlashStringHelper *detail) {
  Serial.print(F("GRIPPER_STOP_DETAIL reason=hard_current state="));
  Serial.print(gripperWatchName());
  Serial.print(F(" detail="));
  Serial.print(detail);
  Serial.print(F(" current_raw="));
  Serial.print(currentRaw);
  Serial.print(F(" current_ma_aprox="));
  Serial.println(currentRaw * 6.5f, 1);
  torqueOffForOvercurrent();
}

void processGripperWatch() {
  if (gripperWatch == WATCH_NONE || gripperClosing) return;
  unsigned long now = millis();
  if (now - gripperLastWatchMs < GRIPPER_WATCH_INTERVAL_MS) return;
  gripperLastWatchMs = now;
  unsigned long elapsed = now - gripperWatchStartedMs;

  int signedCurrentRaw = servoBus.ReadCurrent(6);
  if (signedCurrentRaw == -1 && servoBus.getErr() != 0) {
    gripperWatchReadFailures++;
    if (gripperWatchReadFailures >= GRIPPER_READ_FAILURE_LIMIT) {
      Serial.print(F("GRIPPER_STOP reason=communication_error detail=ReadCurrent_sin_respuesta_al_vigilar state="));
      Serial.println(gripperWatchName());
      releaseGripperAfterFailure("communication_error");
    }
    return;
  }
  gripperWatchReadFailures = 0;
  int currentRaw = abs(signedCurrentRaw);
  if (gripperTelemetry) {
    Serial.print(F("GRIPPER_TELEM state="));
    Serial.print(gripperWatchName());
    Serial.print(F(" current_raw="));
    Serial.print(currentRaw);
    Serial.print(F(" elapsed_ms="));
    Serial.println(elapsed);
  }

  bool inrush = gripperWatch == WATCH_MOVING && elapsed < GRIPPER_MOVE_INRUSH_IGNORE_MS;
  if (!inrush && currentRaw >= GRIPPER_HARD_CURRENT_RAW) {
    gripperWatchHardSamples++;
    if (gripperWatchHardSamples >= GRIPPER_HARD_CURRENT_SAMPLES) {
      stopWatchWithTorqueOff(currentRaw, F("sobre_el_limite_duro"));
      return;
    }
  } else {
    gripperWatchHardSamples = 0;
  }

  if (gripperWatch == WATCH_HOLDING) {
    gripperWatchHighSamples = currentRaw >= GRIPPER_HOLD_LIMIT_RAW ? gripperWatchHighSamples + 1 : 0;
    if (gripperWatchHighSamples < GRIPPER_HOLD_LIMIT_SAMPLES) return;
    gripperWatchHighSamples = 0;
    if (gripperReliefs >= GRIPPER_HOLD_MAX_RELIEFS) {
      stopWatchWithTorqueOff(currentRaw, F("sigue_alta_despues_de_aflojar"));
      return;
    }
    gripperReliefs++;
    gripperCommandDegrees = min(GRIPPER_OPEN_DEGREES, gripperCommandDegrees + GRIPPER_HOLD_RELIEF_DEGREES);
    servoBus.WritePosEx(6, (int)degToPos(toRealServoDegrees(6, gripperCommandDegrees)), 120, 15);
    Serial.print(F("GRIPPER_HOLD_RELIEF current_raw="));
    Serial.print(currentRaw);
    Serial.print(F(" current_ma_aprox="));
    Serial.print(currentRaw * 6.5f, 1);
    Serial.print(F(" logical_deg="));
    Serial.print(gripperCommandDegrees, 1);
    Serial.print(F(" relief="));
    Serial.println(gripperReliefs);
    return;
  }

  if (elapsed >= GRIPPER_MOVE_WATCH_MS) {
    gripperWatch = WATCH_NONE;
    Serial.println(F("GRIPPER_WATCH_END state=moving"));
  }
}

void processGripperClosing() {
  if (!gripperClosing) return;
  unsigned long now = millis();
  if (now - gripperLastStepMs < GRIPPER_STEP_INTERVAL_MS) return;
  gripperLastStepMs = now;

  // ReadCurrent usa el signo para indicar el sentido de la corriente/movimiento.
  // Un valor negativo no es necesariamente un error de comunicacion. La
  // libreria expone getErr() para distinguir un fallo real de una lectura
  // valida con signo negativo.
  int signedCurrentRaw = servoBus.ReadCurrent(6);
  if (signedCurrentRaw == -1 && servoBus.getErr() != 0) {
    Serial.print(F("GRIPPER_STOP reason=communication_error detail=ReadCurrent_sin_respuesta elapsed_ms="));
    Serial.println(now - gripperCloseStartedMs);
    gripperClosing = false;
    gripperHighCurrentSamples = 0;
    releaseGripperAfterFailure("communication_error");
    return;
  }
  int currentRaw = abs(signedCurrentRaw);

  // La sobrecorriente va antes que el tiempo maximo: si coinciden, gana el corte.
  if (currentRaw >= GRIPPER_HARD_CURRENT_RAW) {
    Serial.print(F("GRIPPER_STOP_DETAIL reason=hard_current threshold_raw="));
    Serial.print(GRIPPER_HARD_CURRENT_RAW);
    Serial.print(F(" threshold_ma_aprox="));
    Serial.println(GRIPPER_HARD_CURRENT_RAW * 6.5f, 1);
    gripperClosing = false;
    gripperHighCurrentSamples = 0;
    torqueOffForOvercurrent();
    return;
  }

  if (now - gripperCloseStartedMs >= GRIPPER_CLOSE_TIMEOUT_MS) {
    Serial.print(F("GRIPPER_STOP_DETAIL reason=timeout elapsed_ms="));
    Serial.println(now - gripperCloseStartedMs);
    freezeGripperAtCurrentPosition("timeout", currentRaw);  // sigue vigilada al sostener
    return;
  }

  if (now - gripperCloseStartedMs >= GRIPPER_INRUSH_IGNORE_MS && currentRaw >= gripperContactCurrentRaw) {
    gripperHighCurrentSamples++;
  } else {
    gripperHighCurrentSamples = 0;
  }

  if (gripperHighCurrentSamples >= GRIPPER_CONTACT_SAMPLES) {
    Serial.print(F("GRIPPER_STOP_DETAIL reason=contact samples="));
    Serial.print(gripperHighCurrentSamples);
    Serial.print(F(" threshold_raw="));
    Serial.print(gripperContactCurrentRaw);
    Serial.print(F(" threshold_ma_aprox="));
    Serial.println(gripperContactCurrentRaw * 6.5f, 1);
    freezeGripperAtCurrentPosition("contact", currentRaw);
    return;
  }

  if (gripperCommandDegrees <= GRIPPER_CLOSED_DEGREES) {
    // La consigna puede llegar a 0 antes que el servo fisico. No detener el
    // cierre hasta que el encoder confirme que la garra realmente llego.
    int actualPosition = servoBus.ReadPos(6);
    if (actualPosition < 0) {
      Serial.println(F("GRIPPER_STOP reason=communication_error detail=ReadPos_sin_respuesta_en_limite"));
      gripperClosing = false;
      gripperHighCurrentSamples = 0;
      releaseGripperAfterFailure("communication_error");
      return;
    }

    float actualLogicalDegrees = constrain(
      posToDeg(actualPosition) - 10.0f,
      GRIPPER_CLOSED_DEGREES,
      GRIPPER_OPEN_DEGREES
    );

    if (actualLogicalDegrees <= GRIPPER_CLOSED_DEGREES + GRIPPER_CLOSED_TOLERANCE_DEGREES) {
      Serial.print(F("GRIPPER_STOP_DETAIL reason=closed_limit actual_logical_deg="));
      Serial.println(actualLogicalDegrees, 1);
      freezeGripperAtCurrentPosition("closed_limit", currentRaw);
      return;
    }

    // Mantener la orden de cierre mientras el servo alcanza la consigna.
    float closedRealDegrees = toRealServoDegrees(6, GRIPPER_CLOSED_DEGREES);
    servoBus.WritePosEx(6, (int)degToPos(closedRealDegrees), 220, 18);
    return;
  }

  gripperCommandDegrees = max(GRIPPER_CLOSED_DEGREES, gripperCommandDegrees - GRIPPER_STEP_DEGREES);
  float realDegrees = toRealServoDegrees(6, gripperCommandDegrees);
  servoBus.WritePosEx(6, (int)degToPos(realDegrees), 220, 18);
}

void reportWaypoint(
  const __FlashStringHelper *event, uint32_t id, const __FlashStringHelper *reason
) {
  Serial.print(event);
  Serial.print(F(" id="));
  Serial.print(id);
  if (reason != nullptr) {
    Serial.print(F(" reason="));
    Serial.print(reason);
  }
  Serial.println();
}

void finishWaypoint(const __FlashStringHelper *event, const __FlashStringHelper *reason) {
  const uint32_t id = waypoint.id;
  waypoint.id = 0;
  reportWaypoint(event, id, reason);
}

void skipWaypointSpaces(const char *&cursor) {
  while (isspace((unsigned char)*cursor)) cursor++;
}

bool readWaypointInt(const char *&cursor, long minimum, long maximum, long &value) {
  skipWaypointSpaces(cursor);
  char *end;
  errno = 0;
  value = strtol(cursor, &end, 10);
  if (end == cursor || errno == ERANGE || (*end && !isspace((unsigned char)*end)) ||
      value < minimum || value > maximum) {
    return false;
  }
  cursor = end;
  return true;
}

bool readWaypointDegrees(const char *&cursor, float &value) {
  skipWaypointSpaces(cursor);
  char *end;
  errno = 0;
  value = strtof(cursor, &end);
  if (end == cursor || errno == ERANGE || (*end && !isspace((unsigned char)*end)) ||
      !isfinite(value)) {
    return false;
  }
  cursor = end;
  return true;
}

void handleWaypointLine(const String &line) {
  bool cancel = line.startsWith("waypoint cancel ");
  bool keepalive = line.startsWith("waypoint keepalive ");
  const char *cursor = line.c_str() + (cancel ? 16 : keepalive ? 19 : 9);
  long id;
  if (!readWaypointInt(cursor, 1, 2147483647L, id)) {
    reportWaypoint(F("WAYPOINT_REJECTED"), 0, F("formato"));
    return;
  }
  skipWaypointSpaces(cursor);
  if (cancel || keepalive) {
    if (*cursor) {
      reportWaypoint(F("WAYPOINT_REJECTED"), id, F("formato"));
    } else if (waypoint.id == (uint32_t)id) {
      if (cancel) {
        finishWaypoint(F("WAYPOINT_CANCELLED"), nullptr);
      } else {
        waypoint.heartbeatAtMs = millis();
      }
    }
    return;
  }
  if (waypoint.id != 0) {
    reportWaypoint(waypoint.id == (uint32_t)id ? F("WAYPOINT_BUSY") : F("WAYPOINT_REJECTED"),
                   id, F("ocupado"));
    return;
  }
  if (lastWaypointId == (uint32_t)id) {
    reportWaypoint(F("WAYPOINT_REJECTED"), id, F("id_repetido"));
    return;
  }
  if (gripperClosing || gripperWatchReadFailures || gripperWatchHardSamples || gripperWatchHighSamples) {
    reportWaypoint(F("WAYPOINT_REJECTED"), id, F("gripper_activa"));
    return;
  }

  Waypoint candidate = {};
  uint8_t usedMotors = 0;
  while (*cursor && candidate.size < WAYPOINT_ARM_MOTORS) {
    long count;
    if (!readWaypointInt(cursor, 1, 2, count)) {
      reportWaypoint(F("WAYPOINT_REJECTED"), id, F("formato"));
      return;
    }
    WaypointGroup &group = candidate.groups[candidate.size++];
    group.size = count;
    for (uint8_t i = 0; i < group.size; i++) {
      long motor;
      float degrees;
      if (!readWaypointInt(cursor, 1, WAYPOINT_ARM_MOTORS, motor)) {
        reportWaypoint(F("WAYPOINT_REJECTED"), id, F("motores"));
        return;
      }
      uint8_t mask = 1 << motor;
      if (usedMotors & mask) {
        reportWaypoint(F("WAYPOINT_REJECTED"), id, F("motores"));
        return;
      }
      if (!readWaypointDegrees(cursor, degrees)) {
        reportWaypoint(F("WAYPOINT_REJECTED"), id, F("formato"));
        return;
      }
      float minimum, maximum;
      if (!getUserAngleLimits(motor, minimum, maximum) || degrees < minimum || degrees > maximum) {
        reportWaypoint(F("WAYPOINT_REJECTED"), id, F("fuera_de_rango"));
        return;
      }
      usedMotors |= mask;
      group.ids[i] = motor;
      group.degrees[i] = degrees;
    }
    skipWaypointSpaces(cursor);
  }
  if (*cursor || candidate.size == 0) {
    reportWaypoint(F("WAYPOINT_REJECTED"), id, F("formato"));
    return;
  }
  const unsigned long now = millis();
  candidate.id = id;
  candidate.heartbeatAtMs = now;
  candidate.groupStartedAtMs = now;
  candidate.lastPollAtMs = now - WAYPOINT_POLL_MS;
  candidate.gripperReliefsAtStart = gripperReliefs;
  waypoint = candidate;
  lastWaypointId = id;
  reportWaypoint(F("WAYPOINT_ACCEPTED"), id, nullptr);
}

void processWaypoint() {
  if (waypoint.id == 0) return;

  unsigned long now = millis();
  if (now - waypoint.heartbeatAtMs >= WAYPOINT_HEARTBEAT_MS) {
    finishWaypoint(F("WAYPOINT_REJECTED"), F("heartbeat"));
    return;
  }
  if (now - waypoint.groupStartedAtMs >= WAYPOINT_GROUP_TIMEOUT_MS) {
    finishWaypoint(F("WAYPOINT_REJECTED"), F("timeout"));
    return;
  }
  if (gripperClosing || gripperReliefs != waypoint.gripperReliefsAtStart ||
      gripperWatchReadFailures || gripperWatchHardSamples || gripperWatchHighSamples) {
    finishWaypoint(F("WAYPOINT_REJECTED"), F("gripper_activa"));
    return;
  }
  if (Serial.available() > 0 || inputLine.length() > 0 || discardInputLine ||
      now - waypoint.lastPollAtMs < WAYPOINT_POLL_MS) {
    return;
  }
  waypoint.lastPollAtMs = now;

  float pose[MAX_ID + 1];
  bool quiet = true;
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (servoBus.FeedBack(id) < 0) {
      finishWaypoint(F("WAYPOINT_REJECTED"), F("telemetria"));
      return;
    }
    int position = servoBus.ReadPos(-1);
    int speed = servoBus.ReadSpeed(-1);
    int moving = servoBus.ReadMove(-1);
    if (position < 0 || position > 4095 || (moving != 0 && moving != 1)) {
      finishWaypoint(F("WAYPOINT_REJECTED"), F("telemetria"));
      return;
    }
    int torque = servoBus.readByte(id, SMS_STS_TORQUE_ENABLE);
    if (torque != 1) {
      finishWaypoint(F("WAYPOINT_REJECTED"), torque < 0 ? F("telemetria") : F("torque"));
      return;
    }
    if (id == 6 && waypoint.nextGroup > 0 && (moving != 0 || abs(speed) > WAYPOINT_QUIET_SPEED_RAW)) {
      finishWaypoint(F("WAYPOINT_REJECTED"), F("gripper_activa"));
      return;
    }
    pose[id] = posToDeg(position);
    quiet = quiet && moving == 0 && abs(speed) <= WAYPOINT_QUIET_SPEED_RAW;
  }

  if (waypoint.nextGroup > 0) {
    const WaypointGroup &previous = waypoint.groups[waypoint.nextGroup - 1];
    for (uint8_t i = 0; i < previous.size; i++) {
      quiet = quiet && fabsf(pose[previous.ids[i]] - previous.degrees[i]) <= WAYPOINT_POSITION_TOLERANCE;
    }
  }
  if (!quiet) {
    waypoint.quietSamples = 0;
    return;
  }
  if (waypoint.quietSamples < WAYPOINT_QUIET_SAMPLES) waypoint.quietSamples++;
  if (waypoint.quietSamples < WAYPOINT_QUIET_SAMPLES) return;
  if (waypoint.nextGroup == waypoint.size) {
    finishWaypoint(F("WAYPOINT_DONE"), nullptr);
    return;
  }

  const WaypointGroup &group = waypoint.groups[waypoint.nextGroup];
  const uint8_t last = group.size - 1;
  String reason;
  // Para un solo motor, repetir su destino en esta validacion revisa el
  // recorrido sin aplicar el ajuste automatico de una orden manual.
  if (!validatePairTrajectory(
        group.ids[0], group.degrees[0], group.ids[last], group.degrees[last], reason, false, true
      )) {
    finishWaypoint(F("WAYPOINT_REJECTED"), F("trayectoria_insegura"));
    return;
  }
  now = millis();
  if (now - waypoint.heartbeatAtMs >= WAYPOINT_HEARTBEAT_MS) {
    finishWaypoint(F("WAYPOINT_REJECTED"), F("heartbeat"));
    return;
  }
  if (Serial.available() > 0 || now - lastMoveCommandMs < WAYPOINT_SPACING_MS) return;

  // SyncWrite no devuelve confirmacion individual. Leer ambos destinos
  // despues evita seguir la secuencia si un servo no recibio la orden.
  lastMoveCommandMs = now;
  if (group.size == 2) {
    u16 speeds[2];
    writeSynchronizedPair(group.ids[0], group.degrees[0], pose[group.ids[0]],
                          group.ids[1], group.degrees[1], pose[group.ids[1]], speeds);
  } else if (servoBus.WritePosEx(group.ids[0], degToPos(group.degrees[0]),
                                 scaledArmSpeed(armMaxSpeed), armAcc) != 1) {
    finishWaypoint(F("WAYPOINT_REJECTED"), F("escritura"));
    return;
  } else {
    rememberArmGoal(group.ids[0], degToPos(group.degrees[0]), armMaxSpeed);
  }
  for (uint8_t i = 0; i < group.size; i++) {
    if (servoBus.readWord(group.ids[i], SMS_STS_GOAL_POSITION_L) != degToPos(group.degrees[i])) {
      finishWaypoint(F("WAYPOINT_REJECTED"), F("escritura"));
      return;
    }
  }
  waypoint.nextGroup++;
  waypoint.quietSamples = 0;
  waypoint.groupStartedAtMs = millis();
  waypoint.lastPollAtMs = waypoint.groupStartedAtMs;
}

// Piso, autocolision y regla de M4 en 20 puntos de la recta desde la pose medida
// hasta los destinos, como validatePairTrajectory pero con hasta cinco motores.
// M1 y M5 no cambian el modelo plano; M6 queda como esta (sync no lo mueve).
bool validateSyncTrajectory(const uint8_t *ids, const float *degrees, uint8_t count, String &reason) {
  float currentM2 = 0.0f;
  float currentM3 = 0.0f;
  float currentM4 = 0.0f;
  if (!getCurrentPoseDegrees(currentM2, currentM3, currentM4)) {
    reason = "LECTURA_M2_M3_M4";
    return false;
  }
  float m6 = currentGripperDegreesOrNan();
  float targetM2 = currentM2;
  float targetM3 = currentM3;
  float targetM4 = currentM4;
  for (uint8_t i = 0; i < count; i++) {
    if (ids[i] == 2) targetM2 = degrees[i];
    if (ids[i] == 3) targetM3 = degrees[i];
    if (ids[i] == 4) targetM4 = degrees[i];
  }
  float escapeFrom = floorEscapeReference(currentM2, currentM3, currentM4, m6);
  const int INTERPOLATION_STEPS = 20;
  for (int step = 1; step <= INTERPOLATION_STEPS; step++) {
    float t = (float)step / (float)INTERPOLATION_STEPS;
    float m2 = currentM2 + (targetM2 - currentM2) * t;
    float m3 = currentM3 + (targetM3 - currentM3) * t;
    float m4 = currentM4 + (targetM4 - currentM4) * t;
    if (m4 > getDynamicM4Max(m2, m3)) {
      reason = "REGLA_DINAMICA_M4";
      return false;
    }
    if (classifyPoseInvalid(m2, m3, m4, m6, reason, false, escapeFrom)) {
      return false;
    }
  }
  reason = "OK";
  return true;
}

void rejectSync(const __FlashStringHelper *reason, const char *detail = nullptr) {
  Serial.print(F("SYNC_REJECTED reason="));
  Serial.print(reason);
  if (detail != nullptr) {
    Serial.print(F(" detail="));
    Serial.print(detail);
  }
  Serial.println();
}

// sync <dur_ms> <id> <grados> [<id> <grados> ...]
// Hasta cinco motores de M1..M5, cada uno una vez. dur_ms = 0: el motor que mas
// recorre va a `speed max` y los demas proporcionales; dur_ms > 0: se intenta
// llegar en ese tiempo, sin pasar `speed max`. Fuera de rango o trayectoria
// insegura se rechaza entera: no hay ajustes automaticos.
void handleSyncLine(const String &line) {
  const char *cursor = line.c_str() + 5;
  long durationMs;
  if (!readWaypointInt(cursor, 0, SYNC_MAX_DURATION_MS, durationMs)) {
    rejectSync(F("formato"));
    return;
  }
  uint8_t ids[SYNC_MAX_MOTORS];
  float degrees[SYNC_MAX_MOTORS];
  uint8_t count = 0;
  uint8_t usedMotors = 0;
  skipWaypointSpaces(cursor);
  while (*cursor) {
    if (count == SYNC_MAX_MOTORS) {
      rejectSync(F("formato"));
      return;
    }
    long motor;
    float value;
    if (!readWaypointInt(cursor, 1, SYNC_MAX_MOTORS, motor)) {
      rejectSync(F("motores"));
      return;
    }
    uint8_t mask = 1 << motor;
    if (usedMotors & mask) {
      rejectSync(F("motores"));
      return;
    }
    if (!readWaypointDegrees(cursor, value)) {
      rejectSync(F("formato"));
      return;
    }
    float minimum, maximum;
    if (!getUserAngleLimits(motor, minimum, maximum) || value < minimum || value > maximum) {
      rejectSync(F("fuera_de_rango"));
      return;
    }
    usedMotors |= mask;
    ids[count] = motor;
    degrees[count] = value;
    count++;
    skipWaypointSpaces(cursor);
  }
  if (count == 0) {
    rejectSync(F("formato"));
    return;
  }
  unsigned long now = millis();
  if (now - lastMoveCommandMs < MOVE_COOLDOWN_MS) {
    rejectSync(F("cooldown"));
    return;
  }

  float current[SYNC_MAX_MOTORS];
  for (uint8_t i = 0; i < count; i++) {
    if (!getCurrentLogicalDegrees(ids[i], current[i])) {
      rejectSync(F("servo_sin_respuesta"));
      return;
    }
  }
  String safetyReason;
  if (!validateSyncTrajectory(ids, degrees, count, safetyReason)) {
    rejectSync(F("trayectoria_insegura"), safetyReason.c_str());
    return;
  }

  float maxDelta = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    maxDelta = max(maxDelta, fabsf(degrees[i] - current[i]));
  }
  float cruise = armMaxSpeed;
  if (durationMs > 0 && maxDelta > 0.1f) {
    float wanted = (maxDelta / 360.0f * 4095.0f) / (durationMs / 1000.0f);
    cruise = constrain(wanted, (float)SYNC_MIN_SPEED, (float)armMaxSpeed);
  }
  float base[SYNC_MAX_MOTORS];
  u8 syncIds[SYNC_MAX_MOTORS];
  s16 positions[SYNC_MAX_MOTORS];
  u16 speeds[SYNC_MAX_MOTORS];
  u8 accelerations[SYNC_MAX_MOTORS];
  for (uint8_t i = 0; i < count; i++) {
    float share = maxDelta > 0.1f ? fabsf(degrees[i] - current[i]) / maxDelta : 1.0f;
    syncIds[i] = ids[i];
    positions[i] = (s16)degToPos(toRealServoDegrees(ids[i], degrees[i]));
    base[i] = max((float)SYNC_MIN_SPEED, cruise * share);
    speeds[i] = scaledArmSpeed(base[i]);
    accelerations[i] = armAcc;
  }
  servoBus.SyncWritePosEx(syncIds, count, positions, speeds, accelerations);
  for (uint8_t i = 0; i < count; i++) {
    rememberArmGoal(ids[i], positions[i], base[i]);
  }
  lastMoveCommandMs = now;
  pendingMoveReport = false;

  Serial.print(F("SYNC_MOVE_OK n="));
  Serial.print(count);
  Serial.print(F(" dur_ms="));
  Serial.print(durationMs);
  Serial.print(F(" ids="));
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) Serial.print(',');
    Serial.print(ids[i]);
  }
  Serial.print(F(" speeds="));
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) Serial.print(',');
    Serial.print(speeds[i]);
  }
  Serial.println();
}

void printCapabilities() {
  Serial.print(F("FIRMWARE_CAPS sync=1 sync_max_motors="));
  Serial.print(SYNC_MAX_MOTORS);
  Serial.print(F(" sync_max_speed="));
  Serial.print(armMaxSpeed);
  Serial.print(F(" guard=1 speed_hard_max="));
  Serial.println(ARM_SPEED_HARD_MAX);
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("caps")) {
    printCapabilities();
    return;
  }

  if (line == "waypoint") {
    reportWaypoint(F("WAYPOINT_REJECTED"), 0, F("formato"));
    return;
  }
  if (line.startsWith("waypoint ")) {
    handleWaypointLine(line);
    return;
  }

  if (waypoint.id != 0) {
    if (line.equalsIgnoreCase("arm torque off") || line.equalsIgnoreCase("torque off") ||
        line.equalsIgnoreCase("gripper stop") || line.equalsIgnoreCase("parar garra") ||
        line.equalsIgnoreCase("gripper release") || line.equalsIgnoreCase("garra libre")) {
      finishWaypoint(F("WAYPOINT_CANCELLED"), nullptr);
    } else if (!line.equalsIgnoreCase("help") && !line.equalsIgnoreCase("scan") &&
               !line.equalsIgnoreCase("posall") && !line.startsWith("read ") &&
               !line.equalsIgnoreCase("power telem on") && !line.equalsIgnoreCase("power telem off") &&
               !line.equalsIgnoreCase("gripper telem on") && !line.equalsIgnoreCase("gripper telem off")) {
      reportWaypoint(F("WAYPOINT_BUSY"), waypoint.id, F("ocupado"));
      return;
    }
  }

  if (line.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  if (line.equalsIgnoreCase("scan")) {
    scanAll();
    return;
  }

  if (line.equalsIgnoreCase("posall")) {
    printAllPositions();
    return;
  }

  if (line.equalsIgnoreCase("gripper open") || line.equalsIgnoreCase("abrir garra")) {
    openGripper();
    return;
  }

  if (line.equalsIgnoreCase("gripper close") || line.equalsIgnoreCase("cerrar garra")) {
    startClosingGripper();
    return;
  }

  if (line.equalsIgnoreCase("gripper stop") || line.equalsIgnoreCase("parar garra")) {
    stopGripper();
    return;
  }

  if (line.equalsIgnoreCase("gripper release") || line.equalsIgnoreCase("garra libre")) {
    releaseGripperTorque();
    return;
  }

  if (line.equalsIgnoreCase("power guard on") || line.equalsIgnoreCase("power guard off")) {
    powerGuard = line.equalsIgnoreCase("power guard on");
    guardHardSamples = 0;
    guardGoodSamples = 0;
    speedScalePct = 100;
    Serial.print(F("POWER_GUARD="));
    Serial.println(powerGuard ? F("on") : F("off"));
    return;
  }

  if (line.startsWith("power guard ")) {
    int slowDv = 0, holdDv = 0;
    long slowMa = 0, holdMa = 0;
    char extra = 0;
    int parsed = sscanf(line.c_str(), "power guard %d %d %ld %ld %c", &slowDv, &holdDv, &slowMa, &holdMa, &extra);
    if (parsed != 4 || holdDv < 50 || slowDv <= holdDv || slowDv > 130 ||
        slowMa < 100 || holdMa <= slowMa || holdMa > 10000) {
      Serial.println(F("POWER_GUARD_REJECTED reason=fuera_de_rango"));
      return;
    }
    guardSlowDv = slowDv;
    guardHoldDv = holdDv;
    guardSlowMa = slowMa;
    guardHoldMa = holdMa;
    Serial.print(F("POWER_GUARD_LIMITS slow_dv="));
    Serial.print(guardSlowDv);
    Serial.print(F(" hold_dv="));
    Serial.print(guardHoldDv);
    Serial.print(F(" slow_ma="));
    Serial.print(guardSlowMa);
    Serial.print(F(" hold_ma="));
    Serial.println(guardHoldMa);
    return;
  }

  if (line.startsWith("speed max ")) {
    int speed = 0, acc = MOVE_ACC;
    char extra = 0;
    int parsed = sscanf(line.c_str(), "speed max %d %d %c", &speed, &acc, &extra);
    if ((parsed != 1 && parsed != 2) || speed < ARM_SPEED_MIN_SETTING || speed > ARM_SPEED_HARD_MAX ||
        acc < ARM_ACC_MIN_SETTING || acc > ARM_ACC_MAX_SETTING) {
      Serial.println(F("ARM_SPEED_REJECTED reason=fuera_de_rango"));
      return;
    }
    armMaxSpeed = speed;
    armAcc = acc;
    Serial.print(F("ARM_SPEED max="));
    Serial.print(armMaxSpeed);
    Serial.print(F(" acc="));
    Serial.println(armAcc);
    return;
  }

  if (line.equalsIgnoreCase("power telem on") || line.equalsIgnoreCase("power telem off")) {
    powerTelemetry = line.equalsIgnoreCase("power telem on");
    lastPowerTelemMs = 0;
    Serial.print(F("POWER_TELEMETRY="));
    Serial.println(powerTelemetry ? F("on") : F("off"));
    return;
  }

  if (line.equalsIgnoreCase("gripper telem on") || line.equalsIgnoreCase("gripper telem off")) {
    gripperTelemetry = line.equalsIgnoreCase("gripper telem on");
    Serial.print(F("GRIPPER_TELEMETRY="));
    Serial.println(gripperTelemetry ? F("on") : F("off"));
    return;
  }

  if (line.startsWith("gripper current ")) {
    int requested = line.substring(16).toInt();
    if (requested < 20 || requested > 180) {
      Serial.println(F("Umbral invalido. Usa gripper current 20..180."));
    } else {
      gripperContactCurrentRaw = requested;
      Serial.print(F("GRIPPER_CURRENT_THRESHOLD="));
      Serial.println(gripperContactCurrentRaw);
    }
    return;
  }

  if (line.equalsIgnoreCase("arm torque off") || line.equalsIgnoreCase("arm torque on")) {
    bool enable = line.equalsIgnoreCase("arm torque on");
    forgetArmGoals();
    Serial.println(enable ? F("Activando torque del brazo M1..M5...") : F("Desactivando torque del brazo M1..M5... M6 no cambia."));
    for (uint8_t id = MIN_ID; id <= 5; id++) {
      if (!servoPresent(id)) {
        Serial.print(F("ID "));
        Serial.print(id);
        Serial.println(F(": sin respuesta"));
        continue;
      }
      int result = servoBus.EnableTorque(id, enable ? 1 : 0);
      Serial.print(F("ID "));
      Serial.print(id);
      Serial.print(F(": torque "));
      Serial.print(enable ? F("ON") : F("OFF"));
      Serial.print(F(" resultado="));
      Serial.println(result);
    }
    Serial.println(enable ? F("ARM_TORQUE_ON_OK") : F("ARM_TORQUE_OFF_OK"));
    return;
  }

  if (line.equalsIgnoreCase("torque off") || line.equalsIgnoreCase("torque on")) {
    bool enable = line.equalsIgnoreCase("torque on");
    forgetArmGoals();
    if (!enable) {
      // Emergencia: que el cierre o la vigilancia no vuelvan a mover M6.
      if (gripperClosing) {
        Serial.println(F("GRIPPER_STOP reason=torque_off detail=se_cancelo_el_cierre"));
      }
      gripperClosing = false;
      gripperHighCurrentSamples = 0;
      gripperWatch = WATCH_NONE;
    }
    Serial.println(enable ? F("Activando torque global M1..M6...") : F("Desactivando torque global M1..M6..."));
    for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
      if (!servoPresent(id)) {
        Serial.print(F("ID "));
        Serial.print(id);
        Serial.println(F(": sin respuesta"));
        continue;
      }
      int result = servoBus.EnableTorque(id, enable ? 1 : 0);
      Serial.print(F("ID "));
      Serial.print(id);
      Serial.print(F(": torque "));
      Serial.print(enable ? F("ON") : F("OFF"));
      Serial.print(F(" resultado="));
      Serial.println(result);
    }
    Serial.println(enable ? F("TORQUE_ON_OK") : F("TORQUE_OFF_OK"));
    return;
  }

  if (line.startsWith("read ")) {
    int id = line.substring(5).toInt();
    readOneServo((uint8_t)id);
    printDivider();
    return;
  }

  if (line == "sync") {
    rejectSync(F("formato"));
    return;
  }
  if (line.startsWith("sync ")) {
    handleSyncLine(line);
    return;
  }

  if (line.startsWith("bpair ")) {
    int id1 = 0;
    int id2 = 0;
    float degrees1 = 0.0f;
    float degrees2 = 0.0f;
    int parsed = sscanf(line.c_str(), "bpair %d %f %d %f", &id1, &degrees1, &id2, &degrees2);
    if (parsed != 4) {
      Serial.println(F("PAIR_REJECTED reason=formato_bpair"));
      return;
    }
    moveTwoServos(
      (uint8_t)id1, degrees1, (uint8_t)id2, degrees2, true
    );
    return;
  }

  if (line.startsWith("bmove ")) {
    int id = 0;
    float degrees = 0.0f;
    int parsed = sscanf(line.c_str(), "bmove %d %f", &id, &degrees);
    if (parsed != 2) {
      Serial.println(F("Bloqueado: formato_bmove"));
      return;
    }
    moveServoToDegrees((uint8_t)id, degrees, true);
    return;
  }

  if (line.startsWith("pair ")) {
    int id1 = 0;
    int id2 = 0;
    float degrees1 = 0.0f;
    float degrees2 = 0.0f;
    int parsed = sscanf(line.c_str(), "pair %d %f %d %f", &id1, &degrees1, &id2, &degrees2);
    if (parsed != 4) {
      Serial.println(F("PAIR_REJECTED reason=formato usa: pair 1 90 2 80"));
      return;
    }
    moveTwoServos((uint8_t)id1, degrees1, (uint8_t)id2, degrees2);
    return;
  }

  if (line.startsWith("motor")) {
    int spaceIndex = line.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.println(F("Formato: motor2 90"));
      return;
    }
    int id = line.substring(5, spaceIndex).toInt();
    float degrees = line.substring(spaceIndex + 1).toFloat();
    moveServoToDegrees((uint8_t)id, degrees);
    return;
  }

  int spaceIndex = line.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println(F("Comando invalido. Usa 'help'."));
    return;
  }

  String idPart = line.substring(0, spaceIndex);
  String degPart = line.substring(spaceIndex + 1);
  int id = idPart.toInt();
  float degrees = degPart.toFloat();

  moveServoToDegrees((uint8_t)id, degrees);
}

void setup() {
  Serial.begin(DEBUG_BAUD);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;

  printDivider();
  Serial.println(F("Control sincronizado: pair (dos motores) y sync (hasta cinco del brazo)"));
  Serial.println(F("Speed fijo = 450"));
  Serial.println(F("Acc fijo = 45"));
  Serial.println(F("sync: hasta cinco motores M1..M5 por comando, mismas reglas de piso y colision."));
  Serial.println(F("Lectura automatica de posiciones cada 0.05s (20 Hz)."));
  printHelp();
}

void readPower(PowerReading &reading) {
  reading.totalRaw = 0;
  reading.minDv = 1000;
  reading.minId = 0;
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    int currentRaw = servoBus.ReadCurrent(id);
    if (currentRaw == -1 && servoBus.getErr() != 0) {
      currentRaw = -1;
    } else {
      currentRaw = abs(currentRaw);
      reading.totalRaw += currentRaw;
    }
    reading.raw[id] = currentRaw;
    int voltage = servoBus.ReadVoltage(id);
    if (voltage > 0 && voltage < reading.minDv) {
      reading.minDv = voltage;
      reading.minId = id;
    }
  }
  if (reading.minId == 0) reading.minDv = -1;
}

void printPowerTelemetry(unsigned long now, const PowerReading &reading) {
  // Una linea por muestra: corrientes crudas (6,5 mA por unidad, -1 sin lectura),
  // total en mA y el voltaje mas bajo del bus en decimas de voltio, con su ID.
  Serial.print(F("POWER_TELEM ms="));
  Serial.print(now);
  Serial.print(F(" I="));
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (id > MIN_ID) Serial.print(',');
    Serial.print(reading.raw[id]);
  }
  Serial.print(F(" total_ma="));
  Serial.print(reading.totalRaw * CURRENT_MA_PER_UNIT, 0);
  Serial.print(F(" v_min_dv="));
  Serial.print(reading.minDv);
  Serial.print(F(" v_id="));
  Serial.println(reading.minId);
}

void printGuardState(const char *prefix, const __FlashStringHelper *reason, const PowerReading &reading) {
  Serial.print(prefix);
  Serial.print(F(" reason="));
  Serial.print(reason);
  Serial.print(F(" v_min_dv="));
  Serial.print(reading.minDv);
  Serial.print(F(" total_ma="));
  Serial.println(reading.totalRaw * CURRENT_MA_PER_UNIT, 0);
}

// Reenvia los destinos recientes de M1..M5 que todavia no se alcanzaron, con la
// velocidad escalada: mismos destinos, mismas proporciones entre motores.
void reissueArmGoals() {
  u8 ids[5];
  s16 positions[5];
  u16 speeds[5];
  u8 accelerations[5];
  uint8_t count = 0;
  unsigned long now = millis();
  for (uint8_t id = 1; id <= 5; id++) {
    if (!armGoalValid[id] || now - armGoalAtMs[id] > ARM_GOAL_REISSUE_MS) continue;
    int position = servoBus.ReadPos(id);
    if (position < 0 || abs(position - armGoal[id]) <= 8) continue;
    ids[count] = id;
    positions[count] = armGoal[id];
    speeds[count] = scaledArmSpeed(armBaseSpeed[id]);
    accelerations[count] = armAcc;
    count++;
  }
  if (count > 0) servoBus.SyncWritePosEx(ids, count, positions, speeds, accelerations);
}

void setSpeedScale(uint8_t pct, const __FlashStringHelper *reason, const PowerReading &reading) {
  if (pct == speedScalePct) return;
  speedScalePct = pct;
  Serial.print(F("POWER_GUARD scale="));
  Serial.print(speedScalePct);
  printGuardState("", reason, reading);
  reissueArmGoals();
}

// Frena M1..M5 donde estan: el destino pasa a ser la posicion actual. No corta el
// torque (el brazo no cae ni suelta el cubo). La PC lo toma como falla y se detiene.
void holdArm(const __FlashStringHelper *reason, const PowerReading &reading) {
  u8 ids[5];
  s16 positions[5];
  u16 speeds[5];
  u8 accelerations[5];
  uint8_t count = 0;
  for (uint8_t id = 1; id <= 5; id++) {
    int position = servoBus.ReadPos(id);
    if (position < 0) continue;
    ids[count] = id;
    positions[count] = position;
    speeds[count] = armMaxSpeed;
    accelerations[count] = armAcc;
    count++;
  }
  if (count > 0) servoBus.SyncWritePosEx(ids, count, positions, speeds, accelerations);
  forgetArmGoals();
  if (waypoint.id != 0) finishWaypoint(F("WAYPOINT_CANCELLED"), F("power_guard"));
  printGuardState("POWER_GUARD_HOLD", reason, reading);
}

void evaluatePowerGuard(const PowerReading &reading) {
  long totalMa = (long)(reading.totalRaw * CURRENT_MA_PER_UNIT);
  bool lowHold = reading.minDv > 0 && reading.minDv < guardHoldDv;
  bool lowSlow = reading.minDv > 0 && reading.minDv < guardSlowDv;
  bool hold = lowHold || totalMa > guardHoldMa;
  bool slow = hold || lowSlow || totalMa > guardSlowMa;
  const __FlashStringHelper *reason = (lowHold || (lowSlow && !hold)) ? F("voltaje") : F("corriente");
  if (hold) {
    if (++guardHardSamples >= GUARD_HOLD_SAMPLES) {
      guardHardSamples = 0;
      holdArm(reason, reading);
    }
  } else {
    guardHardSamples = 0;
  }
  if (slow) {
    guardGoodSamples = 0;
    setSpeedScale(max((int)GUARD_SCALE_MIN_PCT, speedScalePct / 2), reason, reading);
  } else if (speedScalePct < 100 && ++guardGoodSamples >= GUARD_RECOVER_SAMPLES) {
    guardGoodSamples = 0;
    setSpeedScale(min(100, speedScalePct + GUARD_SCALE_STEP_UP_PCT), F("normal"), reading);
  }
}

void processPower() {
  if (!powerTelemetry && !powerGuard) return;
  unsigned long now = millis();
  if (now - lastPowerTelemMs < POWER_SAMPLE_INTERVAL_MS) return;
  lastPowerTelemMs = now;
  PowerReading reading;
  readPower(reading);
  if (powerTelemetry) printPowerTelemetry(now, reading);
  if (powerGuard) evaluatePowerGuard(reading);
}

void loop() {
  processGripperClosing();
  processGripperWatch();
  autoPrintPositionsIfNeeded();
  processPower();
  processPendingMoveReport();

  for (uint8_t count = 0; count < 128 && Serial.available() > 0; count++) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (!discardInputLine && inputLine.length() > 0) handleLine(inputLine);
      inputLine = "";
      discardInputLine = false;
    } else if (!discardInputLine) {
      if (inputLine.length() < 128) {
        inputLine += c;
      } else {
        inputLine = "";
        discardInputLine = true;
      }
    }
  }
  processWaypoint();
}
