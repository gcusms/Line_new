#include <iostream>
#include <fmt/core.h>
#include "devices/serial/serial.hpp"
#include "thread"
#include "solvePnP/solvePnP.hpp"
#include "infer/detector.h"
#include <librealsense2/rs.hpp>
#include "utils.hpp"
#include <unistd.h>
#include <memory>
#include <chrono>
#include "cv-helpers.hpp"

#define WINDOW_SIZE_WIDTH 400
#define WINDOW_SIZE_HEIGHT 800
#define WINDOW_NAME "WOLF"

auto pnp = std::make_shared<solvepnp::PnP>(
      fmt::format("{}{}", CONFIG_FILE_PATH, "/d435i.xml"),
      fmt::format("{}{}", CONFIG_FILE_PATH, "/pnp_config.xml"));

using namespace rs2;


void uartReadThread(const std::shared_ptr<RoboSerial> &serial,
                    RoboInf &robo_inf) {
  while (true) try {
      serial->ReceiveInfo(robo_inf);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (const std::exception &e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
      std::this_thread::sleep_for(std::chrono::seconds(5));
        fmt::print("[{}] read serial excepted to many times, sleep 10s.\n",
                   idntifier_red);
        serial_read_excepted_times = 0;
      }

      fmt::print("[{}] serial exception: {} serial restarting...\n",
                 idntifier_red, e.what());
      std::this_thread::sleep_for(std::chrono::seconds(5));
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
void imageProcess_thread(cv::Mat &frame,RoboInf &robo_inf,const std::shared_ptr<RoboSerial> &serial)
{
    fmt::print("Robot_R2_CameraTest\n");
    // the part of the inferences 
    Detector* detector = new Detector;
    string xml_path = "../model/best.xml";
    detector->init(xml_path,0.15,0.75);
    // Mat src = imread("../model/13.jpg");
    cv::Rect object_2d_rect;
    cv::Rect object_3d_rect(0, 0, 140, 140);
    cv::Point2f pnp_angle;
    cv::Point3f pnp_coordinate_mm;
    float pnp_depth;

    pipeline pipe;                           // 创建数据管道
    // start() 函数返回数据管道的 profile
    pipeline_profile profile = pipe.start(); 

    while (true)
    {
        // 堵塞程序直到新的一帧捕获
        frameset frameset = pipe.wait_for_frames();  

        // 获取颜色图
        rs2::video_frame video_src = frameset.get_color_frame(); 

        // 获取深度图
        rs2::depth_frame depth_src = frameset.get_depth_frame(); 

        // 获取深度图的尺寸，用于确定测距中心点
        float width_ = depth_src.get_width();
        float height_ = depth_src.get_height();

        // 获取颜色图的尺寸，用于转成 Mat 格式并显示
        const int color_width = video_src.as<video_frame>().get_width();
        const int color_height = video_src.as<video_frame>().get_height();

        // 转成 Mat 类型
        Mat frame(Size(color_width, color_height), CV_8UC3,             
        (void*)video_src.get_data(),Mat::AUTO_STEP);
        // 定义并获取距离数据（单位： m ）
        // float distance = depth_src.get_distance(width_ / 2, height_ / 2); 

        // 输出距离数据
        // cout << "distance:" << distance << endl;  
        if (!frame.empty()) {
            Mat osrc = frame.clone();
            resize(osrc,osrc,Size(640,640));
            vector<Detector::Object> detected_objects;

            // auto start = chrono::high_resolution_clock::now();
            auto t = (double)cv::getTickCount();
            detector->process_frame(osrc,detected_objects);
            t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
            auto fps = 1.0 / t;
            fmt::print("FPS:{}\n",fps);
            // auto end = chrono::high_resolution_clock::now();
            // std::chrono::duration<double> diff = end - start;
            // cout<<"use "<<diff.count()<<" s" << endl;

            Rect rect = cv::Rect(0,0,0,0);
            for(size_t i=0;i< detected_objects.size();++i){
                int xmin = detected_objects[i].rect.x;
                int ymin = detected_objects[i].rect.y;
                int width = detected_objects[i].rect.width;
                int height = detected_objects[i].rect.height;
                rect = cv::Rect(xmin, ymin, width, height);//左上坐标（x,y）和矩形的长(x)宽(y)
                RoboCatchCmdUartBuff reset_buff;
                pnp->solvePnP(object_3d_rect, rect, pnp_angle, // to do:test the real data
                              pnp_coordinate_mm, pnp_depth);
                reset_buff.yaw_angle = cube_target_distance_offset - pnp_angle.x;
                float img_sub = 0.0f;
                float disatnce_get = 0.0f;
                cv::circle(osrc,cv::Point(xmin + width * 0.5, ymin + height *0.5),2,
                cv::Scalar(255,255,255),-1,cv::LINE_8);
                disatnce_get = depth_src.get_distance(xmin + width * 0.2, ymin + height *0.2);
                if (!judgeTheCube(rect,frame,robo_inf,reset_buff,img_sub))  // 不在合适的范围里面
                {
#ifdef DRAW_RECT == 1
    cv::rectangle(osrc, 
    Rect(xmin, ymin, width, height), 
    cv::Scalar(0, 0, 0),
    2,
    LINE_8,
    0);
  putText(osrc,detected_objects[i].name +"dis"+std::to_string(disatnce_get),
    Point(xmin,ymin - 10),cv::FONT_HERSHEY_COMPLEX,
    0.7,
    cv::Scalar(0, 0, 0),
    0.5,
    cv::LINE_4);

#endif
                reset_buff.yaw_angle = img_sub * 0.01;
                serial->write((uint8_t *)&reset_buff, sizeof(reset_buff));
                  continue;
                }
                reset_buff.yaw_angle = img_sub * 0.01;
                
                if (detected_objects[i].id == 0 || detected_objects[i].id == 3){ // 正面
                    reset_buff.cube_state = 0x01;
                } else if (detected_objects[i].id == 1 || detected_objects[i].id == 4){ // 反面
                  reset_buff.cube_state = 0x02;
                } else if (detected_objects[i].id == 2 || detected_objects[i].id == 5) {  // 竖直
                  reset_buff.cube_state = 0x03;
                }
                serial->write((uint8_t *)&reset_buff, sizeof(reset_buff));
#ifdef DRAW_RECT == 1
cv::rectangle(osrc, 
              Rect(xmin, ymin, width, height), 
              cv::Scalar(255, 0, 255),
              2,
              LINE_8,
              0);
putText(osrc,detected_objects[i].name +" "+std::to_string(disatnce_get),
        Point(xmin,ymin - 10),cv::FONT_HERSHEY_COMPLEX,
        0.7,
        cv::Scalar(255,0,255),
        0.5,
        cv::LINE_4);

#endif
            }
            cv::cvtColor(osrc,osrc,cv::COLOR_RGB2BGR);
            cv::line(osrc,cv::Point(osrc.cols* 0.53,0),cv::Point(osrc.cols * 0.53,osrc.rows),
                    cv::Scalar(0,255,255),2,cv::LINE_8);
            cv::line(osrc,cv::Point(osrc.cols* 0.57,0),cv::Point(osrc.cols * 0.57,osrc.rows),
                    cv::Scalar(0,255,255),2,cv::LINE_8);
            cv::imshow("result",osrc);
            // cv::imwrite("block_test.jpg",osrc);
            cv::waitKey(1);
            // imshow("frame", frame); // 显示
        }
    }
}
int main()
{

    auto serial = std::make_shared<RoboSerial>("/dev/ros_tty", 115200);
    RoboInf robo_inf;
    std::thread uart_thread(uartThread, std::ref(robo_inf), std::ref(serial));
    uart_thread.detach();
    cv::Mat src_f;
    std::thread imageThread(imageProcess_thread,std::ref(src_f),std::ref(robo_inf),std::ref(serial));
    imageThread.detach();

    if (getchar()) {
      uart_thread.~thread();
      // testThread.~thread();
      imageThread.~thread();
    }
    getchar();
    return 0;
}


