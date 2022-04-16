#pragma once
#include <atomic>
#include <opencv4/opencv2/highgui/highgui.hpp>


#define AUTO_MODE 0x01
#define MANUAL_MODE 0x02
#define DETECT_MODE 0x03
#define JUDGE_MODE 0x04
#define NOTHING 0x05

#define CUBE_1 0x01 // min
#define CUBE_2 0x02
#define CUBE_3 0x03
#define CUBE_4 0x04
#define CUBE_5 0x05
#define CUBE_UNCERTAIN 0X06
#define CUBE_UP    0x01
#define CUBE_DOWN  0x02
#define CUBE_STAND 0x03

#define SPIN_SIGN         0x01
#define GO_SIGN           0X02
#define CATCH_SIGN        0X03
#define RETURN_CUBE_STATE 0X04

float distance_set = 0.0f;

enum CatchMode {
  off = 0,
  spin,
  go,
  catch_cube,
  detect_mode
};

struct RoboInf {
  std::atomic<uint8_t> mode {0x00};
  std::atomic<CatchMode> catch_cube_mode_status {CatchMode::off};
};

// send R2 spin command
struct RoboSpinCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = SPIN_SIGN;
  float yaw_angle = 0.f;
  uint8_t E_flag = 'E';
} __attribute__((packed));

// send R2 spin command
struct RoboGoCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = GO_SIGN;
  float distance = 0.f;
  uint8_t E_flag = 'E';
} __attribute__((packed));

// send R2 catch command
// cube_state: 0x01 - yellow, 0x02 - white, 0x03 - stand
// cube_type: 0x01 - 0x05
struct RoboCatchCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = CATCH_SIGN;
  uint8_t cube_state = 0x00;
  uint8_t cube_type = 0x00;
  uint8_t E_flag = 'E';
} __attribute__((packed));

// send R2 cube status
// 0x01 white 0x02 yellow 
// cube_type: 0x01 - 0x05
struct RoboCubeStateUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cmd_type = RETURN_CUBE_STATE;
  uint8_t cube_status = 0x00;
  uint8_t cube_type = 0x00;
  uint8_t E_flag = 'E';
} __attribute__((packed));

//uart recive
struct RoboInfUartBuff {
  uint8_t mode = NOTHING;
} __attribute__((packed));


