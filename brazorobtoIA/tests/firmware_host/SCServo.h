// Sustitutos mínimos de Arduino y de la biblioteca SCServo para compilar
// firmware/firmware_secuencias.ino en el PC y probar su lógica sin hardware.
// No reproducen tiempos reales del bus ni la electrónica del servo.
#pragma once

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

using std::isnan;

typedef uint8_t u8;
typedef int16_t s16;
typedef uint16_t u16;

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define SERIAL_8N1 0
#define F(text) (text)

template <typename A, typename B>
auto max(A a, B b) -> decltype(a + b) { return a > b ? a : b; }
template <typename A, typename B>
auto min(A a, B b) -> decltype(a + b) { return a < b ? a : b; }
template <typename V, typename L, typename H>
V constrain(V value, L low, H high) { return value < low ? low : (value > high ? high : value); }

extern unsigned long host_millis;
inline unsigned long millis() { return host_millis; }

class String {
 public:
  String() {}
  String(const char *text) : value(text) {}
  String(const std::string &text) : value(text) {}
  unsigned int length() const { return (unsigned int)value.size(); }
  const char *c_str() const { return value.c_str(); }
  void trim() {
    size_t start = value.find_first_not_of(" \t\r\n");
    size_t end = value.find_last_not_of(" \t\r\n");
    value = start == std::string::npos ? "" : value.substr(start, end - start + 1);
  }
  bool equalsIgnoreCase(const char *other) const {
    if (value.size() != std::strlen(other)) return false;
    for (size_t i = 0; i < value.size(); i++) {
      if (std::tolower((unsigned char)value[i]) != std::tolower((unsigned char)other[i])) return false;
    }
    return true;
  }
  bool startsWith(const char *prefix) const { return value.rfind(prefix, 0) == 0; }
  String substring(unsigned int from) const { return from >= value.size() ? String() : String(value.substr(from)); }
  String substring(unsigned int from, unsigned int to) const {
    return from >= value.size() ? String() : String(value.substr(from, to - from));
  }
  int indexOf(char c) const {
    size_t index = value.find(c);
    return index == std::string::npos ? -1 : (int)index;
  }
  long toInt() const { return std::strtol(value.c_str(), nullptr, 10); }
  float toFloat() const { return std::strtof(value.c_str(), nullptr); }
  String &operator+=(char c) { value += c; return *this; }
  String &operator=(const char *text) { value = text; return *this; }
  bool operator==(const char *text) const { return value == text; }
  std::string value;
};

class HostSerial {
 public:
  void begin(long) {}
  int available() { return (int)input.size(); }
  int read() {
    if (input.empty()) return -1;
    char c = input.front();
    input.pop_front();
    return c;
  }
  void print(const char *text) { output += text; }
  void print(const String &text) { output += text.value; }
  void print(char c) { output += c; }
  void print(int value) { output += std::to_string(value); }
  void print(long value) { output += std::to_string(value); }
  void print(unsigned int value) { output += std::to_string(value); }
  void print(unsigned long value) { output += std::to_string(value); }
  void print(double value, int digits = 2) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
    output += buffer;
  }
  template <typename T>
  void println(T value) { print(value); output += "\n"; }
  void println(double value, int digits) { print(value, digits); output += "\n"; }
  void println() { output += "\n"; }
  std::deque<char> input;
  std::string output;
};
extern HostSerial Serial;

class HardwareSerial {
 public:
  explicit HardwareSerial(int) {}
  void begin(long, int, int, int) {}
};

// Servo simulado. La posición sigue a la consigna a velocidad fija; la
// corriente de M6 la fija cada prueba.
struct HostServo {
  bool present = true;
  bool torque = false;
  double position = 2048.0;
  double target = 2048.0;
  double block_at = -1.0;  // Posición donde un objeto detiene el cierre (<0: sin objeto).
  int current = 0;
  int current_read_failures = 0;  // Lecturas de corriente siguientes que fallan.
  int torque_write_failures = 0;  // Escrituras de torque siguientes sin confirmar.
};

class SMS_STS {
 public:
  HardwareSerial *pSerial = nullptr;
  HostServo servos[7];
  std::vector<std::string> calls;
  int err = 0;

  int getErr() { return err; }
  int Ping(u8 id) {
    err = servos[id].present ? 0 : 1;
    return servos[id].present ? id : -1;
  }
  bool fail(u8 id) { return !servos[id].present; }
  int ReadPos(u8 id) {
    if (id == 6) calls.push_back("ReadPos");
    if (fail(id)) { err = 1; return -1; }
    err = 0;
    return (int)std::lround(servos[id].position);
  }
  int ReadCurrent(u8 id) {
    if (id == 6) calls.push_back("ReadCurrent");
    if (servos[id].current_read_failures > 0) {
      servos[id].current_read_failures--;
      err = 1;
      return -1;
    }
    if (fail(id)) { err = 1; return -1; }
    err = 0;
    return servos[id].current;
  }
  int ReadVoltage(u8) { err = 0; return 120; }
  int ReadTemper(u8) { err = 0; return 30; }
  int EnableTorque(u8 id, u8 enable) {
    if (id == 6) calls.push_back(enable ? "TorqueOn" : "TorqueOff");
    if (!servos[id].present) return 0;
    if (servos[id].torque_write_failures > 0) {
      servos[id].torque_write_failures--;
      return 0;
    }
    servos[id].torque = enable != 0;
    return 1;
  }
  int WritePosEx(u8 id, s16 position, u16, u8) {
    if (id == 6) calls.push_back("WritePos");
    if (!servos[id].present) return 0;
    servos[id].target = position;
    servos[id].torque = true;
    return 1;
  }
  void SyncWritePosEx(u8 ids[], u8 count, s16 positions[], u16[], u8[]) {
    for (u8 i = 0; i < count; i++) {
      servos[ids[i]].target = positions[i];
      servos[ids[i]].torque = true;
    }
  }
  // Avanza la simulación 1 ms: ~0,5 unidades de encoder por ms hacia la consigna.
  void step() {
    for (int id = 1; id <= 6; id++) {
      HostServo &servo = servos[id];
      if (!servo.torque) continue;
      double delta = servo.target - servo.position;
      double move = delta > 0 ? std::fmin(delta, 0.5) : std::fmax(delta, -0.5);
      double next = servo.position + move;
      if (servo.block_at >= 0 && move < 0 && next < servo.block_at) next = servo.block_at;
      servo.position = next;
    }
  }
};
