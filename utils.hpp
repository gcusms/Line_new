#pragma once
#include <atomic>
#include <opencv4/opencv2/highgui/highgui.hpp>


#define CUBE_1 0x01 // cube_1(biggest)
#define CUBE_2 0x02
#define CUBE_3 0x03
#define CUBE_4 0x04
#define CUBE_5 0x05
#define CUBE_UP    0x01
#define CUBE_DOWN  0x02
#define CUBE_STAND 0x03

#define SPIN_SIGN         0x01
#define GO_SIGN           0X02
#define CATCH_SIGN        0X03
#define RETURN_CUBE_STATE 0X04


constexpr float cube_target_distance_offset = 13.5f;


#define DRAW_RECT 1
enum CatchMode {
  wait = 0,
  spin,
  go,
  catch_cube
};

enum RobotColor{
  RED = 0,
  BLUE ,
};

/**
 * @brief 块的状态
 * FRONT 正面 RECVERSE 反面 VERTICAL
 */
enum CubeStatus
{
  FRONT, 
  REVERSE,
  VERTICAL
};

struct RoboInf {
  std::atomic<bool> auto_catch_cube_mode {false}; // 自动模式（自动对位并进行识别）
  std::atomic<bool> manual_catch_cube_mode {false}; // 手动模式（没有自动对位）
  std::atomic<bool> detect_cube_mode {false}; // 识别立起积木模式（仅进行积木状态识别）
  std::atomic<int> robot_self_color{RED};
  std::atomic<CatchMode> catch_cube_mode_status {CatchMode::wait};
  std::atomic<double> value_d;  // 测试
};

bool judgeTheCube(cv::Rect &rect_input,const cv::Mat src_img_input,
                      RoboInf &robo_inf,RoboCatchCmdUartBuff &reset_buff)
  {
    int center_x_judge = rect_input.x + rect_input.width *0.5;
    if(center_x_judge > src_img_input.cols * 0.3 && 
       center_x_judge < src_img_input.cols *0.7) {
      reset_buff.cube_needen = 0x01;
      return true;
    }
  reset_buff.cube_needen = 0x00;
  return false;
}

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
// cube_needen: 0x00 move 0x01 no move
struct RoboCatchCmdUartBuff {
  uint8_t S_flag = 'S';
  uint8_t cube_needen = 0x00;
  uint8_t cmd_type = CATCH_SIGN;
  uint8_t cube_state = 0x00;
  uint8_t cube_type = 0x00;
  float yaw_angle = 0.f;
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
  bool auto_catch_cube_mode {false};
  bool manual_catch_cube_mode {false};
  bool detect_cube_mode {false};
  bool signal_finish{false};
  double value_dd;
  int robot_clor{RED};

} __attribute__((packed));

