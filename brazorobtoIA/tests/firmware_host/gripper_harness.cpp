// Pruebas de la vigilancia de corriente de la garra (M6) del firmware, en el PC.
// Compila firmware/firmware_secuencias.ino con los sustitutos de SCServo.h y
// recorre casos con corrientes y fallos de bus simulados. No prueba hardware.
#include "SCServo.h"

unsigned long host_millis = 0;
HostSerial Serial;

#include "firmware_preprocesado.ino"

#include <functional>
#include <iostream>

static int failures = 0;
static std::string current_case;

#define CHECK(condition, message)                                              \
  do {                                                                         \
    if (!(condition)) {                                                        \
      failures++;                                                              \
      std::cout << "FAIL " << current_case << ": " << message << "\n";         \
    }                                                                          \
  } while (0)

static HostServo &gripper() { return servoBus.servos[6]; }

static double logicalToPos(float logical) { return degToPos(logical + 10.0f); }

static void reset_world() {
  host_millis = 1000;
  Serial.output.clear();
  servoBus = SMS_STS();
  for (int id = 1; id <= 6; id++) {
    servoBus.servos[id].position = servoBus.servos[id].target = 2048;
  }
  gripper().position = gripper().target = logicalToPos(140.0f);
  gripper().torque = true;
  gripperMode = GRIPPER_IDLE;
  gripperCommandDegrees = GRIPPER_OPEN_DEGREES;
  gripperCloseStartedMs = 0;
  gripperLastCheckMs = 0;
  gripperLastReportMs = 0;
  resetGripperCounters();
  gripperContactCurrentRaw = 92;
}

static void run(unsigned long ms, const std::function<void()> &each = nullptr) {
  for (unsigned long i = 0; i < ms; i++) {
    host_millis++;
    servoBus.step();
    if (each) each();
    loop();
  }
}

static void send(const char *command) { handleLine(String(command)); }

static bool has(const std::string &text) { return Serial.output.find(text) != std::string::npos; }

static void begin(const char *name) {
  reset_world();
  current_case = name;
}

static void test_idle_is_not_supervised() {
  begin("idle_sin_vigilancia");
  run(500);
  int reads = 0;
  for (auto &call : servoBus.calls) reads += call == "ReadCurrent";
  CHECK(reads == 0, "no debe leer corriente sin orden previa para M6");
}

static void close_on_cube(double cube_logical) {
  double block = logicalToPos((float)cube_logical);
  gripper().block_at = block;
  gripper().current = 30;
  send("gripper close");
  run(6000, [block] { gripper().current = gripper().position <= block + 0.5 ? 100 : 30; });
}

static void test_contact_then_hold_is_supervised() {
  begin("contacto_y_sujecion_vigilada");
  close_on_cube(40.0);
  CHECK(has("GRIPPER_HOLD reason=contact"), "debe detectar contacto");
  CHECK(gripperMode == GRIPPER_POSITION, "tras el contacto la garra queda en POSITION");
  Serial.output.clear();
  run(1000);
  CHECK(has("GRIPPER_I raw=100 mode=position"), "debe informar la corriente al sostener");
  CHECK(!has("GRIPPER_TORQUE_OFF"), "100 unidades sostenidas no cortan");
}

static void test_single_spike_while_holding_does_not_trip() {
  begin("pico_aislado_al_sostener");
  close_on_cube(40.0);
  Serial.output.clear();
  gripper().current = 250;
  run(100);
  gripper().current = 100;
  run(500);
  CHECK(!has("GRIPPER_TORQUE_OFF"), "una sola lectura alta al sostener no corta");
  CHECK(gripperMode == GRIPPER_POSITION, "sigue en POSITION");
}

static void test_hard_current_while_holding_trips_first() {
  begin("sobrecorriente_al_sostener");
  close_on_cube(40.0);
  servoBus.calls.clear();
  Serial.output.clear();
  gripper().current = 250;
  run(300);
  CHECK(has("GRIPPER_TORQUE_OFF reason=hard_current current_raw=250 result=1 attempts=1"), "debe cortar torque");
  CHECK(gripperMode == GRIPPER_RELEASED, "queda RELEASED");
  CHECK(!gripper().torque, "M6 sin torque");
  size_t off = 0;
  for (size_t i = 0; i < servoBus.calls.size(); i++) {
    if (servoBus.calls[i] == "TorqueOff") { off = i; break; }
  }
  CHECK(off > 0 && servoBus.calls[off - 1] == "ReadCurrent",
        "el apagado va inmediatamente después de la lectura, sin leer ni escribir posición");
  Serial.output.clear();
  servoBus.calls.clear();
  run(500);
  int gripper_ops = 0;
  for (auto &call : servoBus.calls) gripper_ops += call == "ReadCurrent" || call == "WritePos";
  CHECK(gripper_ops == 0, "RELEASED no vuelve a leer corriente ni escribir M6");
}

static void test_hard_current_while_opening_trips() {
  begin("sobrecorriente_al_abrir");
  close_on_cube(40.0);
  gripper().block_at = -1;
  send("gripper open");
  CHECK(has("GRIPPER_OPEN_OK"), "abre");
  gripper().current = 260;
  run(300);
  CHECK(has("GRIPPER_TORQUE_OFF reason=hard_current"), "la apertura también se vigila");
}

static void test_hard_current_has_priority_over_timeout() {
  begin("sobrecorriente_antes_que_timeout");
  gripper().block_at = logicalToPos(60.0f);
  gripper().current = 50;
  send("gripper close");
  unsigned long start = gripperCloseStartedMs;
  run(14990);
  CHECK(gripperMode == GRIPPER_CLOSING, "sigue cerrando antes del timeout");
  run(200, [start] { if (millis() - start >= GRIPPER_CLOSE_TIMEOUT_MS) gripper().current = 250; });
  CHECK(has("GRIPPER_TORQUE_OFF reason=hard_current"), "la sobrecorriente gana");
  CHECK(!has("reason=timeout"), "no debe informar timeout");
}

static void test_timeout_keeps_supervising() {
  begin("timeout_sigue_vigilado");
  gripper().block_at = logicalToPos(60.0f);
  gripper().current = 50;
  send("gripper close");
  run(15200);
  CHECK(has("GRIPPER_HOLD reason=timeout"), "informa timeout");
  CHECK(gripperMode == GRIPPER_POSITION, "queda en POSITION");
  Serial.output.clear();
  gripper().current = 250;
  run(300);
  CHECK(has("GRIPPER_TORQUE_OFF reason=hard_current"), "después del timeout se sigue vigilando");
}

static void test_closed_limit_without_object() {
  begin("cierre_sin_objeto");
  gripper().current = 30;
  send("gripper close");
  run(12000);
  CHECK(has("GRIPPER_HOLD reason=closed_limit"), "informa cierre sin objeto");
  CHECK(!has("reason=contact"), "no confunde con contacto");
}

static void test_transient_read_failures_do_not_stop() {
  begin("fallos_de_lectura_aislados");
  gripper().block_at = logicalToPos(60.0f);
  gripper().current = 30;
  send("gripper close");
  run(1000);
  float before = gripperCommandDegrees;
  gripper().current_read_failures = 2;
  run(80);
  CHECK(gripperCommandDegrees == before, "sin lectura válida no avanza la consigna");
  CHECK(has("GRIPPER_READ_FAIL detail=ReadCurrent_sin_respuesta count=2"), "informa los fallos");
  CHECK(!has("GRIPPER_STOP reason=communication_error"), "dos fallos no cortan");
  run(200);
  CHECK(gripperMode == GRIPPER_CLOSING, "retoma el cierre");
  CHECK(gripperCommandDegrees < before, "la consigna vuelve a avanzar");
}

static void test_repeated_read_failures_release_torque() {
  begin("fallos_de_lectura_seguidos");
  close_on_cube(40.0);
  Serial.output.clear();
  gripper().current_read_failures = 3;
  run(400);
  CHECK(has("GRIPPER_STOP reason=communication_error"), "informa el error");
  CHECK(has("GRIPPER_TORQUE_OFF reason=communication_error"), "desenergiza M6");
  CHECK(gripperMode == GRIPPER_RELEASED, "queda RELEASED");
}

static void test_dead_servo_reports_unconfirmed_torque_off() {
  begin("servo_sin_respuesta");
  close_on_cube(40.0);
  Serial.output.clear();
  gripper().present = false;
  run(400);
  CHECK(has("GRIPPER_TORQUE_OFF reason=communication_error current_raw=-1 result=0 attempts=3"), "reintenta 3 veces");
  CHECK(has("GRIPPER_FAULT reason=torque_off_sin_confirmar source=communication_error"), "informa que no se confirmó");
}

static void test_torque_off_retry() {
  begin("reintento_de_apagado");
  close_on_cube(40.0);
  Serial.output.clear();
  gripper().torque_write_failures = 1;
  gripper().current = 250;
  run(300);
  CHECK(has("result=1 attempts=2"), "confirma en el segundo intento");
  CHECK(!has("GRIPPER_FAULT"), "sin falla si se confirmó");
}

static void test_repeated_close_is_ignored() {
  begin("pulsacion_repetida");
  gripper().block_at = logicalToPos(60.0f);
  gripper().current = 30;
  send("gripper close");
  unsigned long start = gripperCloseStartedMs;
  run(300);
  send("gripper close");
  CHECK(has("GRIPPER_CLOSE_IGNORED reason=already_closing"), "ignora la segunda orden");
  CHECK(gripperCloseStartedMs == start, "no reinicia el cronómetro");
}

static void test_global_torque_off_cancels_closing() {
  begin("torque_off_cancela_cierre");
  gripper().block_at = logicalToPos(60.0f);
  gripper().current = 30;
  send("gripper close");
  run(300);
  send("torque off");
  CHECK(has("GRIPPER_STOP reason=torque_off"), "informa la cancelación");
  CHECK(gripperMode == GRIPPER_RELEASED, "queda RELEASED");
  servoBus.calls.clear();
  run(500);
  int writes = 0;
  for (auto &call : servoBus.calls) writes += call == "WritePos";
  CHECK(writes == 0, "no sigue cerrando después del apagado");
}

static void test_global_torque_on_supervises() {
  begin("torque_on_vigila");
  send("torque on");
  CHECK(gripperMode == GRIPPER_POSITION, "torque global ON vigila M6");
  gripper().current = 250;
  run(300);
  CHECK(has("GRIPPER_TORQUE_OFF reason=hard_current"), "corta también fuera de una orden de garra");
}

static void test_manual_angle_cancels_closing_and_supervises() {
  begin("angulo_manual_m6");
  gripper().block_at = logicalToPos(60.0f);
  gripper().current = 30;
  send("gripper close");
  run(300);
  host_millis += 100;  // Fuera del cooldown de movimientos.
  send("6 100");
  CHECK(has("GRIPPER_STOP reason=manual_position_command"), "cancela el cierre");
  CHECK(gripperMode == GRIPPER_POSITION, "queda vigilada");
}

int main() {
  test_idle_is_not_supervised();
  test_contact_then_hold_is_supervised();
  test_single_spike_while_holding_does_not_trip();
  test_hard_current_while_holding_trips_first();
  test_hard_current_while_opening_trips();
  test_hard_current_has_priority_over_timeout();
  test_timeout_keeps_supervising();
  test_closed_limit_without_object();
  test_transient_read_failures_do_not_stop();
  test_repeated_read_failures_release_torque();
  test_dead_servo_reports_unconfirmed_torque_off();
  test_torque_off_retry();
  test_repeated_close_is_ignored();
  test_global_torque_off_cancels_closing();
  test_global_torque_on_supervises();
  test_manual_angle_cancels_closing_and_supervises();
  std::cout << (failures == 0 ? "OK" : "FALLAS") << " " << failures << "\n";
  return failures == 0 ? 0 : 1;
}
