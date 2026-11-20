// Stub minimo de Arduino para compilar firmware_secuencias.ino en la PC.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <string>
#include <algorithm>
using std::isnan; using std::isfinite;
typedef uint8_t u8; typedef int16_t s16; typedef uint16_t u16;
#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define SERIAL_8N1 0
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))
template <class A, class B> auto min(A a, B b) -> decltype(a < b ? a : b) { return a < b ? a : b; }
template <class A, class B> auto max(A a, B b) -> decltype(a > b ? a : b) { return a > b ? a : b; }
template <class T, class L, class H> T constrain(T v, L lo, H hi) { return v < lo ? (T)lo : (v > hi ? (T)hi : v); }
extern unsigned long fake_millis;
inline unsigned long millis() { return fake_millis; }
class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c ? c : "") {}
  String(const std::string &x) : s(x) {}
  unsigned length() const { return s.size(); }
  const char *c_str() const { return s.c_str(); }
  bool startsWith(const char *p) const { return s.rfind(p, 0) == 0; }
  bool equalsIgnoreCase(const char *o) const {
    std::string a = s, b = o;
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) if (tolower(a[i]) != tolower(b[i])) return false;
    return true;
  }
  void trim() {
    size_t a = s.find_first_not_of(" \t\r\n"); size_t b = s.find_last_not_of(" \t\r\n");
    s = a == std::string::npos ? "" : s.substr(a, b - a + 1);
  }
  int indexOf(char c) const { size_t i = s.find(c); return i == std::string::npos ? -1 : (int)i; }
  String substring(unsigned a) const { return String(a >= s.size() ? std::string() : s.substr(a)); }
  String substring(unsigned a, unsigned b) const { return String(s.substr(a, b - a)); }
  long toInt() const { return atol(s.c_str()); }
  float toFloat() const { return (float)atof(s.c_str()); }
  bool operator==(const char *o) const { return s == o; }
  String &operator+=(char c) { s += c; return *this; }
  String &operator=(const char *c) { s = c; return *this; }
};
class HardwareSerialBase {
 public:
  std::string out;
  std::string in;
  void begin(long, int = 0, int = 0, int = 0) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; int c = (unsigned char)in[0]; in.erase(0, 1); return c; }
  void print(const __FlashStringHelper *t) { out += reinterpret_cast<const char *>(t); }
  void print(const char *t) { out += t; }
  void print(const String &t) { out += t.s; }
  void print(char c) { out += c; }
  void print(int v) { out += std::to_string(v); }
  void print(unsigned v) { out += std::to_string(v); }
  void print(long v) { out += std::to_string(v); }
  void print(unsigned long v) { out += std::to_string(v); }
  void print(double v, int digits = 2) { char b[64]; snprintf(b, sizeof b, "%.*f", digits, v); out += b; }
  template <class T> void println(T v) { print(v); out += "\r\n"; }
  void println(double v, int d) { print(v, d); out += "\r\n"; }
  void println() { out += "\r\n"; }
};
class HardwareSerial : public HardwareSerialBase { public: HardwareSerial(int = 0) {} };
extern HardwareSerial Serial;
