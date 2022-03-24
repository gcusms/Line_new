#include <opencv4/opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fmt/core.h>
#include "devices/serial/serial.hpp"
#include "thread"
#include "pthread.h"
#include "infer/detector.h"

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

  while (cv::waitKey(1) != 'q')
  {
    try
    {   
      RoboSpinCmdUartBuff uart_temp_struct;
      uart_temp_struct.yaw_angle = 1.0;
      serial->write((uint8_t *)&uart_temp_struct, sizeof(uart_temp_struct));
      fmt::print("[{}]:SENDING!!!\n",idntifier_green);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }

  }
  
}


int main()
{

    auto serial = std::make_shared<RoboSerial>("/dev/ttyUSB0", 115200);
    RoboInf robo_inf;
    // std::thread uart_thread(uartThread, std::ref(robo_inf), std::ref(serial));
    // uart_thread.detach();


    // the part of the inferences 
    Detector* detector = new Detector;
    string xml_path = "../model/best.xml";
    detector->init(xml_path,0.3,0.5);
    Mat src = imread("../model/13.jpg");
    Mat osrc = src.clone();
    resize(osrc,osrc,Size(640,640));
    vector<Detector::Object> detected_objects;
    auto start = chrono::high_resolution_clock::now();
    detector->process_frame(src,detected_objects);
    auto end = chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    cout<<"use "<<diff.count()<<" s" << endl;
    for(int i=0;i<detected_objects.size();++i){
         int xmin = detected_objects[i].rect.x;
        int ymin = detected_objects[i].rect.y;
        int width = detected_objects[i].rect.width;
        int height = detected_objects[i].rect.height;
        Rect rect(xmin, ymin, width, height);//左上坐标（x,y）和矩形的长(x)宽(y)
        cv::rectangle(osrc, rect, Scalar(255, 0, 255),2, LINE_8,0);
        putText(osrc,detected_objects[i].name,
                Point(rect.x,rect.y - 10),cv::FONT_HERSHEY_COMPLEX,
                1.2,
                cv::Scalar(255,0,255),
                0.5,
                cv::LINE_4);
    }
    imshow("result",osrc);
    cv::waitKey(0);
    if (static_cast<char>(cv::waitKey(1)) == 'q') {
      // uart_thread.~thread();
    }
    return 0;
}


void uiShow()
{
    MES m;
    // serial 
    // Show everything on the screen
    cv::Mat frame = cv::Mat(WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT, CV_8UC3);
    double value = 5.0;
    cvui::init(WINDOW_NAME);
    while (true)
    {
      frame = cv::Scalar(49, 52, 49);
      cvui::printf(frame, 10, 300, 0.8, 0x00ffff, "k click count: %d", value);
      // robo_inf.value_d.store((rand() + .0));
      m.points.push_back(rand() + .0);

      fmt::print("[{}]:{}🎄\n",idntifier_red,.0);
      if(m.points.size() >= 30) {
          m.points.erase(m.points.begin()); //删除第一个元素
      }
      cvui::sparkline(frame,m.points,0,10,800,200,0xff00ff);
      cvui::update();
      cv::imshow(WINDOW_NAME, frame);
    }
}