#include <unistd.h>
#include <cstdint>
#include <iostream>
#include <map>
#include "joystick.h"

#define GAMEPAD_TYPE 1 // 1: XBOX, 0: SWITCH
#define MAX_AXES_VALUE 32768
#define MIN_AXES_VALUE -32768
using namespace std;

namespace
{
constexpr uint16_t kR1Bit = 1u << 0;
constexpr uint16_t kL1Bit = 1u << 1;
constexpr uint16_t kStartBit = 1u << 2;
constexpr uint16_t kSelectBit = 1u << 3;
constexpr uint16_t kR2Bit = 1u << 4;
constexpr uint16_t kL2Bit = 1u << 5;
constexpr uint16_t kF1Bit = 1u << 6;
constexpr uint16_t kF2Bit = 1u << 7;
constexpr uint16_t kABit = 1u << 8;
constexpr uint16_t kBBit = 1u << 9;
constexpr uint16_t kXBit = 1u << 10;
constexpr uint16_t kYBit = 1u << 11;
constexpr uint16_t kUpBit = 1u << 12;
constexpr uint16_t kRightBit = 1u << 13;
constexpr uint16_t kDownBit = 1u << 14;
constexpr uint16_t kLeftBit = 1u << 15;

inline void SetBit(uint16_t& value, uint16_t mask, bool enabled)
{
  if (enabled)
  {
    value |= mask;
  }
}
}  // namespace

int main(int argc, char **argv)
{
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  map<string, int> AxisId =
      {
          {"LX", 0}, // Left stick axis x
          {"LY", 1}, // Left stick axis y
          {"RX", 3}, // Right stick axis x
          {"RY", 4}, // Right stick axis y
          {"LT", 2}, // Left trigger
          {"RT", 5}, // Right trigger
          {"DX", 6}, // Directional pad x
          {"DY", 7}, // Directional pad y
      };

  map<string, int> ButtonId =
      {
          {"X", 2},
          {"Y", 3},
          {"B", 1},
          {"A", 0},
          {"LB", 4},
          {"RB", 5},
          {"SELECT", 6},
          {"START", 7},
      };

  while (true)
  {

    // Attempt to sample an event from the joystick
    joystick.getState();

    uint16_t unitree_key = 0;
    SetBit(unitree_key, kR1Bit, joystick.button_[ButtonId["RB"]]);
    SetBit(unitree_key, kL1Bit, joystick.button_[ButtonId["LB"]]);
    SetBit(unitree_key, kStartBit, joystick.button_[ButtonId["START"]]);
    SetBit(unitree_key, kSelectBit, joystick.button_[ButtonId["SELECT"]]);
    SetBit(unitree_key, kR2Bit, joystick.axis_[AxisId["RT"]] > 0);
    SetBit(unitree_key, kL2Bit, joystick.axis_[AxisId["LT"]] > 0);
    SetBit(unitree_key, kF1Bit, false);
    SetBit(unitree_key, kF2Bit, false);
    SetBit(unitree_key, kABit, joystick.button_[ButtonId["A"]]);
    SetBit(unitree_key, kBBit, joystick.button_[ButtonId["B"]]);
    SetBit(unitree_key, kXBit, joystick.button_[ButtonId["X"]]);
    SetBit(unitree_key, kYBit, joystick.button_[ButtonId["Y"]]);
    SetBit(unitree_key, kUpBit, joystick.axis_[AxisId["DY"]] < 0);
    SetBit(unitree_key, kRightBit, joystick.axis_[AxisId["DX"]] > 0);
    SetBit(unitree_key, kDownBit, joystick.axis_[AxisId["DY"]] > 0);
    SetBit(unitree_key, kLeftBit, joystick.axis_[AxisId["DX"]] < 0);

    cout << unitree_key << endl;

    // Restrict rate
    usleep(10000);
  }
  return 0;
};
