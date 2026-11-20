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
  waypoint = {};
  powerGuard = false;
  powerTelemetry = false;
  speedScalePct = 100;
  guardHardSamples = guardGoodSamples = 0;
  forgetArmGoals();
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

  // ----- speed max -----
  resetPose();
  run("speed max 700");
  CHECK(Serial.out.find("ARM_SPEED max=700 acc=45") == 0, "speed max 700");
  run("caps");
  CHECK(Serial.out.find("sync_max_speed=700 guard=1 speed_hard_max=1000") != std::string::npos, "caps con la velocidad nueva");
  servoBus.syncCalls.clear();
  run("sync 0 1 190 2 175");
  CHECK(servoBus.syncCalls.size() == 1 && servoBus.syncCalls[0].speeds[0] == 700, "sync usa speed max");
  for (const char *line : {"speed max 1200", "speed max 50", "speed max 700 200", "speed max 700 5", "speed max x", "speed max 700 45 1"}) {
    run(line);
    CHECK(Serial.out.find("ARM_SPEED_REJECTED reason=fuera_de_rango") == 0, line);
  }
  run("speed max 450 45");
  CHECK(Serial.out.find("ARM_SPEED max=450 acc=45") == 0, "vuelta a 450");

  // ----- power telem: mismo formato que antes -----
  resetPose();
  servoBus.servo[3].current = 10;
  servoBus.servo[4].voltage = 117;
  run("power telem on");
  fake_millis += 200;
  Serial.out.clear();
  processPower();
  CHECK(Serial.out.find("POWER_TELEM ms=") == 0 && Serial.out.find(" I=0,0,10,0,0,0 total_ma=65 v_min_dv=117 v_id=4") != std::string::npos,
        "formato de power telem");
  run("power telem off");

  // ----- supervisor de la fuente -----
  auto sample = [&]() { fake_millis += 100; Serial.out.clear(); processPower(); };
  resetPose();
  servoBus.servo[2].voltage = 90;  // 9,0 V: bajo el umbral lento, sobre el de corte
  sample();
  CHECK(Serial.out.empty(), "apagado al arrancar: no actua");
  run("power guard on");
  CHECK(Serial.out.find("POWER_GUARD=on") == 0, "power guard on");
  servoBus.servo[2].voltage = 120;
  run("sync 0 1 200 2 170");  // destinos lejos: los servos falsos no se mueven
  servoBus.syncCalls.clear();
  servoBus.servo[2].voltage = 90;
  sample();
  CHECK(Serial.out.find("POWER_GUARD scale=50 reason=voltaje v_min_dv=90") == 0, "baja a la mitad por voltaje");
  CHECK(servoBus.syncCalls.size() == 1, "reenvia los destinos pendientes");
  if (servoBus.syncCalls.size() == 1) {
    const SyncCall &c = servoBus.syncCalls[0];
    CHECK(c.ids.size() == 2 && c.positions[0] == toPos(200) && c.positions[1] == toPos(170), "mismos destinos");
    CHECK(c.speeds[0] == 225 && abs(c.speeds[1] - 112) <= 2, "mitad de velocidad, mismas proporciones");
  }
  sample();
  CHECK(Serial.out.find("POWER_GUARD scale=25") == 0, "un escalon mas");
  sample();
  CHECK(Serial.out.empty(), "no baja de 25%");
  servoBus.syncCalls.clear();
  run("sync 0 3 95");
  CHECK(servoBus.syncCalls.size() == 1 && servoBus.syncCalls[0].speeds[0] == 112, "las ordenes nuevas tambien van escaladas");
  servoBus.servo[2].voltage = 120;
  for (int i = 0; i < 9; i++) sample();
  CHECK(Serial.out.empty(), "espera 1 s de lecturas normales");
  sample();
  CHECK(Serial.out.find("POWER_GUARD scale=50 reason=normal") == 0, "sube de a 25%");
  for (int i = 0; i < 20; i++) sample();
  run("caps");
  sample();
  CHECK(speedScalePct == 100, "vuelve a 100%");

  // Corriente total sobre el umbral lento (470 unidades = 3055 mA).
  resetPose();
  run("power guard on");
  speedScalePct = 100;
  servoBus.servo[2].current = 300;
  servoBus.servo[3].current = 170;
  sample();
  CHECK(Serial.out.find("POWER_GUARD scale=50 reason=corriente") == 0, "baja por corriente");

  // Corte: dos lecturas seguidas bajo 8,0 V frenan el brazo donde esta.
  resetPose();
  run("power guard on");
  speedScalePct = 100;
  run("sync 0 1 200");
  servoBus.syncCalls.clear();
  servoBus.servo[5].voltage = 70;
  sample();
  CHECK(Serial.out.find("POWER_GUARD_HOLD") == std::string::npos, "una sola lectura no frena");
  sample();
  CHECK(Serial.out.find("POWER_GUARD_HOLD reason=voltaje v_min_dv=70") != std::string::npos, "frena con dos lecturas");
  bool held = false;
  for (const SyncCall &c : servoBus.syncCalls) {
    if (c.ids.size() == 5 && c.positions[0] == toPos(180)) held = true;  // M1 se queda en 180
  }
  CHECK(held, "el destino de M1..M5 pasa a ser la posicion actual");

  // Con el torque cambiado no se reenvian destinos viejos.
  resetPose();
  run("power guard on");
  speedScalePct = 100;
  run("sync 0 1 200");
  run("arm torque off");
  run("arm torque on");
  servoBus.syncCalls.clear();
  servoBus.servo[2].voltage = 90;
  sample();
  CHECK(Serial.out.find("POWER_GUARD scale=50") == 0 && servoBus.syncCalls.empty(), "sin destinos tras torque");

  // Limites configurables.
  run("power guard 100 85 2500 4000");
  CHECK(Serial.out.find("POWER_GUARD_LIMITS slow_dv=100 hold_dv=85 slow_ma=2500 hold_ma=4000") == 0, "limites");
  for (const char *line : {"power guard 80 85 2500 4000", "power guard 100 40 2500 4000",
                           "power guard 100 85 4000 2500", "power guard 100 85 2500 20000", "power guard 100 85"}) {
    run(line);
    CHECK(Serial.out.find("POWER_GUARD_REJECTED reason=fuera_de_rango") == 0, line);
  }
  run("power guard off");
  CHECK(Serial.out.find("POWER_GUARD=off") == 0 && speedScalePct == 100, "power guard off");

  if (failures == 0) std::cout << "OK: pruebas de sync en la PC" << std::endl;
  return failures == 0 ? 0 : 1;
}
