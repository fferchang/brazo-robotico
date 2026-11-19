#include "firmware_host.cpp"
#include <cassert>
#include <iostream>
unsigned long fake_millis = 10000;
HardwareSerial Serial;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::cerr << "FALLA: " << msg << "\n  salida: " << Serial.out << "\n"; failures++; } } while (0)

static int toPos(float deg) { return (int)lround(deg / 360.0f * 4095.0f); }
// Pose inicial estandar M1..M5 = 180, 180, 90, 180, 263 y M6 cerrada (10 real).
static void resetPose() {
  float pose[7] = {0, 180, 180, 90, 180, 263, 10};
  for (int id = 1; id <= 6; id++) {
    servoBus.servo[id] = FakeServo();
    servoBus.servo[id].pos = servoBus.servo[id].goal = toPos(pose[id]);
  }
  servoBus.syncCalls.clear();
  lastMoveCommandMs = 0;
  fake_millis += 1000;
  Serial.out.clear();
}
static void run(const char *line) { Serial.out.clear(); handleLine(String(line)); }

int main() {
  resetPose();
  run("caps");
  CHECK(Serial.out.find("FIRMWARE_CAPS sync=1 sync_max_motors=5 sync_max_speed=450") == 0, "caps");

  // Cinco motores juntos: un solo SyncWritePosEx, velocidades proporcionales.
  resetPose();
  run("sync 0 1 190 2 175 3 95 4 182 5 260");
  CHECK(Serial.out.find("SYNC_MOVE_OK n=5 dur_ms=0 ids=1,2,3,4,5") == 0, "cinco motores aceptados");
  CHECK(servoBus.syncCalls.size() == 1, "una sola escritura sincronizada");
  if (servoBus.syncCalls.size() == 1) {
    const SyncCall &c = servoBus.syncCalls[0];
    CHECK(c.ids.size() == 5, "cinco ids en la escritura");
    CHECK(c.speeds[0] == 450, "el que mas recorre (M1, 10 grados) va a 450");
    CHECK(abs(c.speeds[1] - 225) <= 3, "M2 (5 grados) a la mitad");
    CHECK(c.positions[0] == toPos(190), "destino de M1");
    for (int s : c.speeds) CHECK(s >= 20 && s <= 450, "velocidad dentro de [20, 450]");
  }

  // Duracion pedida: 10 grados en 1 s son ~114 pasos/s; con 50 ms no se pasa de 450.
  resetPose();
  run("sync 1000 1 190");
  CHECK(servoBus.syncCalls.size() == 1 && abs(servoBus.syncCalls[0].speeds[0] - 113) <= 1, "duracion 1000 ms");
  resetPose();
  run("sync 50 1 190");
  CHECK(servoBus.syncCalls.size() == 1 && servoBus.syncCalls[0].speeds[0] == 450, "techo de velocidad");

  // Un motor que casi no se mueve nunca recibe velocidad 0 (en el STS3215 seria la maxima).
  resetPose();
  run("sync 0 1 190 5 263.05");
  CHECK(servoBus.syncCalls.size() == 1 && servoBus.syncCalls[0].speeds[1] == 20, "piso de velocidad");

  // Enfriamiento compartido con pair y las ordenes de un motor.
  resetPose();
  run("sync 0 1 185");
  fake_millis += 20;
  run("sync 0 1 190");
  CHECK(Serial.out.find("SYNC_REJECTED reason=cooldown") == 0, "cooldown");
  fake_millis += 40;
  run("sync 0 1 190");
  CHECK(Serial.out.find("SYNC_MOVE_OK") == 0, "despues del cooldown");

  // Rechazos de formato, motores y rango: nunca escriben en el bus.
  const char *bad[][2] = {
    {"sync", "formato"}, {"sync 0", "formato"}, {"sync x 1 190", "formato"},
    {"sync 0 6 90", "motores"}, {"sync 0 0 90", "motores"}, {"sync 0 1 190 1 185", "motores"},
    {"sync 0 1 190 2", "formato"}, {"sync 0 1 abc", "formato"}, {"sync 0 1 50", "fuera_de_rango"},
    {"sync 0 2 271", "fuera_de_rango"}, {"sync 0 1 190 2 180 3 90 4 180 5 263 1 180", "formato"},
    {"sync -5 1 190", "formato"}, {"sync 20000 1 190", "formato"}, {"sync 0 1 nan", "formato"},
  };
  for (auto &item : bad) {
    resetPose();
    run(item[0]);
    std::string expected = std::string("SYNC_REJECTED reason=") + item[1];
    CHECK(Serial.out.find(expected) == 0, std::string(item[0]) + " -> " + expected);
    CHECK(servoBus.syncCalls.empty(), std::string(item[0]) + " no escribe");
  }

  // Trayectoria insegura: bajar el brazo hasta la mesa se rechaza entero.
  resetPose();
  run("sync 0 1 190 2 270 3 150 4 180");
  CHECK(Serial.out.find("SYNC_REJECTED reason=trayectoria_insegura detail=") == 0, "trayectoria insegura");
  CHECK(servoBus.syncCalls.empty(), "trayectoria insegura no escribe");
  // La misma regla que pair: si pair la rechaza, sync tambien.
  resetPose();
  // Pose final valida pero el camino pasa por la mesa: se revisa la recta, no solo el final.
  run("pair 2 270 3 276");
  CHECK(Serial.out.find("PAIR_REJECTED reason=trayectoria_insegura") == 0, "pair rechaza el camino");
  resetPose();
  run("sync 0 2 270 3 276");
  CHECK(Serial.out.find("SYNC_REJECTED reason=trayectoria_insegura") == 0, "sync rechaza el mismo camino");
  CHECK(servoBus.syncCalls.empty(), "camino inseguro no escribe");

  // Servo que no responde.
  resetPose();
  servoBus.servo[5].present = false;
  run("sync 0 1 190 5 250");
  CHECK(Serial.out.find("SYNC_REJECTED reason=servo_sin_respuesta") == 0, "servo sin respuesta");

  // Con un punto de firmware activo, sync queda ocupado como pair.
  resetPose();
  run("waypoint 7 1 1 185");
  run("sync 0 1 190");
  CHECK(Serial.out.find("WAYPOINT_BUSY") == 0, "sync con waypoint activo");

  if (failures == 0) std::cout << "OK: pruebas de sync en la PC" << std::endl;
  return failures == 0 ? 0 : 1;
}
