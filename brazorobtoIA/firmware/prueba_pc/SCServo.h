// Bus de servos simulado: posiciones instantaneas y registro de escrituras.
#pragma once
#include "Arduino.h"
#include <vector>
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_GOAL_POSITION_L 42
struct FakeServo { bool present = true; int pos = 2048; int goal = 2048; int torque = 1; int current = 0; };
struct SyncCall { std::vector<int> ids, positions, speeds, accs; };
class SMS_STS {
 public:
  HardwareSerial *pSerial = nullptr;
  FakeServo servo[8];
  std::vector<SyncCall> syncCalls;
  int writes = 0;
  int last = -1;
  int Ping(int id) { return servo[id].present ? id : -1; }
  int ReadPos(int id) { if (id < 0) id = last; return servo[id].present ? servo[id].pos : -1; }
  int ReadVoltage(int) { return 120; }
  int ReadTemper(int) { return 30; }
  int ReadCurrent(int id) { return servo[id].current; }
  int ReadSpeed(int) { return 0; }
  int ReadMove(int) { return 0; }
  int FeedBack(int id) { last = id; return servo[id].present ? 1 : -1; }
  int getErr() { return 0; }
  int EnableTorque(int id, int e) { servo[id].torque = e; return 1; }
  int readByte(int id, int) { return servo[id].torque; }
  int readWord(int id, int) { return servo[id].goal; }
  int WritePosEx(int id, int pos, int, int) { servo[id].goal = pos; writes++; return 1; }
  void SyncWritePosEx(u8 ids[], u8 n, s16 pos[], u16 spd[], u8 acc[]) {
    SyncCall c;
    for (int i = 0; i < n; i++) { c.ids.push_back(ids[i]); c.positions.push_back(pos[i]); c.speeds.push_back(spd[i]); c.accs.push_back(acc[i]); servo[ids[i]].goal = pos[i]; }
    syncCalls.push_back(c);
  }
};
