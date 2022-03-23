#include <opencv4/opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fmt/core.h>
#include "devices/serial/serial.hpp"
#include "thread"
#include "pthread.h"

#define CVUI_IMPLEMENTATION
#include "cvui.hpp"
#include <memory>
#include <chrono>

#define WINDOW_SIZE_WIDTH 400
#define WINDOW_SIZE_HEIGHT 800
#define WINDOW_NAME "WOLF"
struct MES
{
    std::vector<double> points;
};


void uartReadThread(const std::shared_ptr<RoboSerial> &serial,
                    RoboInf &robo_inf) {
  while (true) try {
      serial->ReceiveInfo(robo_inf);
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    } catch (const std::exception &e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
      std::this_thread::sleep_for(std::chrono::microseconds(10000));
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n",
                   idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial exception: {} serial restarting...\n",
                 idntifier_red, e.what());
      std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
}
void uartThread(RoboInf &robo_inf, const std::shared_ptr<RoboSerial> &serial) {
  std::thread uart_read_thread(uartReadThread, std::ref(serial),
                               std::ref(robo_inf));
  uart_read_thread.detach();
}

int main()
{
    MES m;
    // serial 
    auto serial = std::make_shared<RoboSerial>("/dev/ttyUSB0", 115200);
    RoboInf robo_inf;
    std::thread uart_thread(uartThread, std::ref(robo_inf), std::ref(serial));
    uart_thread.detach();
    // Show everything on the screen
    cv::Mat frame = cv::Mat(WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT, CV_8UC3);
    double value = 5.0;
    cvui::init(WINDOW_NAME);
    while (true)
    {
      frame = cv::Scalar(49, 52, 49);
      cvui::printf(frame, 10, 300, 0.8, 0x00ffff, "k click count: %d", value);
      // robo_inf.value_d.store((rand() + .0));
      m.points.push_back(robo_inf.value_d);
      if(m.points.size() >= 30) {
          m.points.erase(m.points.begin()); //删除第一个元素
      }
      cvui::sparkline(frame,m.points,0,10,800,200,0xff00ff);
      cvui::update();
      cv::imshow(WINDOW_NAME, frame);
      if (static_cast<char>(cv::waitKey(1)) == 'q') {
        uart_thread.~thread();
        break;
      }
    }

    return 0;
}