#include <opencv4/opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fmt/core.h>
#include "devices/serial/serial.hpp"
#include "thread"
#include "pthread.h"
#include "infer/detector.h"
#include <librealsense2/rs.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.hpp"
#include <memory>
#include <chrono>

#define WINDOW_SIZE_WIDTH 400
#define WINDOW_SIZE_HEIGHT 800
#define WINDOW_NAME "WOLF"


using namespace rs2;


struct MES
{
    std::vector<double> points;
};


void uartReadThread(const std::shared_ptr<RoboSerial> &serial,
                    RoboInf &robo_inf) {
  while (true) try {
      serial->ReceiveInfo(robo_inf);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (const std::exception &e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
      std::this_thread::sleep_for(std::chrono::seconds(10));
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n",
                   idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial exception: {} serial restarting...\n",
                 idntifier_red, e.what());
      std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}
void uartThread(RoboInf &robo_inf, const std::shared_ptr<RoboSerial> &serial) {
  std::thread uart_read_thread(uartReadThread, std::ref(serial),
                               std::ref(robo_inf));
  uart_read_thread.detach();

  // while (cv::waitKey(1) != 'q')
  // {
  //   try
  //   {   
  //     RoboSpinCmdUartBuff uart_temp_struct;
  //     uart_temp_struct.yaw_angle = 0.50;
  //     serial->write((uint8_t *)&uart_temp_struct, sizeof(uart_temp_struct));
  //     fmt::print("[{}]:SENDING!!!\n",idntifier_green);
  //   }
  //   catch(const std::exception& e)
  //   {
  //     std::cerr << e.what() << '\n';
  //   }
  // }
  
}

void testHtread(const int k)
{
  while (true)
  {
    fmt::print("[{}]:Process Working NOW {}\n",idntifier_green,k);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
void imageProcess_thread(cv::Mat &frame)
{

    // the part of the inferences 
    Detector* detector = new Detector;
    string xml_path = "../model/best.xml";
    detector->init(xml_path,0.15,0.75);
    // Mat src = imread("../model/13.jpg");
    pipeline pipe;                           // 创建数据管道
    // start() 函数返回数据管道的 profile
    pipeline_profile profile = pipe.start(); 
    fmt::print("Robot_R2_CameraTest\n");

    while (true)
    {
        // 堵塞程序直到新的一帧捕获
        frameset frameset = pipe.wait_for_frames();  

        // 获取颜色图
        rs2::video_frame video_src = frameset.get_color_frame(); 

        // 获取深度图
        rs2::depth_frame depth_src = frameset.get_depth_frame(); 

        // 获取深度图的尺寸，用于确定测距中心点
        float width = depth_src.get_width();
        float height = depth_src.get_height();

        // 获取颜色图的尺寸，用于转成 Mat 格式并显示
        const int color_width = video_src.as<video_frame>().get_width();
        const int color_height = video_src.as<video_frame>().get_height();

        // 转成 Mat 类型
        Mat frame(Size(color_width, color_height), CV_8UC3,             
        (void*)video_src.get_data(),Mat::AUTO_STEP);
        // 定义并获取距离数据（单位： m ）
        float distance = depth_src.get_distance(width / 2, height / 2); 

        // 输出距离数据
        cout << "distance:" << distance << endl;  
        if (!frame.empty()) {
            Mat osrc = frame.clone();
            resize(osrc,osrc,Size(640,640));
            vector<Detector::Object> detected_objects;
            auto start = chrono::high_resolution_clock::now();
            detector->process_frame(osrc,detected_objects);
            auto end = chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end - start;
            cout<<"use "<<diff.count()<<" s" << endl;
            for(int i=0;i<detected_objects.size();++i){
                int xmin = detected_objects[i].rect.x;
                int ymin = detected_objects[i].rect.y;
                int width = detected_objects[i].rect.width;
                int height = detected_objects[i].rect.height;
                Rect rect(xmin, ymin, width, height);//左上坐标（x,y）和矩形的长(x)宽(y)
                cv::rectangle(osrc, 
                              rect, 
                              cv::Scalar(255, 0, 255),
                              2,
                              LINE_8,
                              0);
                putText(osrc,detected_objects[i].name,
                        Point(rect.x,rect.y - 10),cv::FONT_HERSHEY_COMPLEX,
                        1.2,
                        cv::Scalar(255,0,255),
                        0.5,
                        cv::LINE_4);
            }
            cv::cvtColor(osrc,osrc,cv::COLOR_RGB2BGR);
            imshow("result",osrc);
            cv::imwrite("/home/sms/tu/block_test.jpg",osrc);

            cv::waitKey(1);
            // imshow("frame", frame); // 显示
        }
    }
}
int main()
{

    auto serial = std::make_shared<RoboSerial>("/dev/ttyUSB0", 115200);
    RoboInf robo_inf;
    std::thread uart_thread(uartThread, std::ref(robo_inf), std::ref(serial));
    uart_thread.detach();
    cv::Mat src_f;
    std::thread imageThread(imageProcess_thread,std::ref(src_f));
    imageThread.detach();
    // std::thread testThread(testHtread,1);
    // testThread.detach();
    // 未开放区域
/*
    while (true)
    {
        // 堵塞程序直到新的一帧捕获
        frameset frameset = pipe.wait_for_frames();  

        // 获取颜色图
        rs2::video_frame video_src = frameset.get_color_frame(); 

        // 获取深度图
        rs2::depth_frame depth_src = frameset.get_depth_frame(); 

        // 获取深度图的尺寸，用于确定测距中心点
        float width = depth_src.get_width();
        float height = depth_src.get_height();

        // 获取颜色图的尺寸，用于转成 Mat 格式并显示
        const int color_width = video_src.as<video_frame>().get_width();
        const int color_height = video_src.as<video_frame>().get_height();

        // 转成 Mat 类型
        Mat frame(Size(color_width, color_height), CV_8UC3,             
        (void*)video_src.get_data(),Mat::AUTO_STEP);

        // 定义并获取距离数据（单位： m ）
        float distance = depth_src.get_distance(width / 2, height / 2); 

        // 输出距离数据
        cout << "distance:" << distance << endl;  
        if (!frame.empty()) {
            Mat osrc = frame.clone();
            resize(osrc,osrc,Size(640,640));
            vector<Detector::Object> detected_objects;
            auto start = chrono::high_resolution_clock::now();
            detector->process_frame(osrc,detected_objects);
            auto end = chrono::high_resolution_clock::now();
            std::chrono::duration<double> diff = end - start;
            cout<<"use "<<diff.count()<<" s" << endl;
            for(int i=0;i<detected_objects.size();++i){
                int xmin = detected_objects[i].rect.x;
                int ymin = detected_objects[i].rect.y;
                int width = detected_objects[i].rect.width;
                int height = detected_objects[i].rect.height;
                Rect rect(xmin, ymin, width, height);//左上坐标（x,y）和矩形的长(x)宽(y)
                cv::rectangle(osrc, 
                              rect, 
                              cv::Scalar(255, 0, 255),
                              2,
                              LINE_8,
                              0);
                putText(osrc,detected_objects[i].name,
                        Point(rect.x,rect.y - 10),cv::FONT_HERSHEY_COMPLEX,
                        1.2,
                        cv::Scalar(255,0,255),
                        0.5,
                        cv::LINE_4);
            }
            imshow("result",osrc);

            // imshow("frame", frame); // 显示
        }
*/

    if (static_cast<char>(cv::waitKey(1)) == 'q') {
      uart_thread.~thread();
      // testThread.~thread();
      imageThread.~thread();
    }
    for (;;)
    { 
      cout << "123" << endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));

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