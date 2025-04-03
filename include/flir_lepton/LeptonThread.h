#ifndef LEPTON_THREAD_H
#define LEPTON_THREAD_H
#include <ctime>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/image.hpp"


#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)

class LeptonThread
{

public:
  LeptonThread();
  LeptonThread(rclcpp::Node::SharedPtr n);
  ~LeptonThread();

  void setLogLevel(uint16_t);
  void useColormap(int);
  void useLepton(int);
  void useSpiSpeedMhz(unsigned int);
  void setAutomaticScalingRange(int);
  void setAutomaticScalingRange();
  void useRangeMinValue(uint16_t);
  void useRangeMaxValue(uint16_t);
  void publishImage();
  void run();
  void setPublisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr);
  void setNode(rclcpp::Node::SharedPtr n);
  bool performFFC(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response
  );

private:
  void log_message(uint16_t, std::string);
  uint16_t loglevel;
  int typeColormap;
  const int *selectedColormap;
  int selectedColormapSize;
  int typeLepton;
  unsigned int spiSpeed;
  bool autoRangeMin;
  bool autoRangeMax;
  uint16_t rangeMin;
  uint16_t rangeMax;
  int myImageWidth;
  int myImageHeight;
  int imgCount;
  cv::Mat myImage;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherImage;
  rclcpp::Node::SharedPtr node;

  uint8_t result[PACKET_SIZE*PACKETS_PER_FRAME];
  uint8_t shelf[4][PACKET_SIZE*PACKETS_PER_FRAME];
  uint16_t *frameBuffer;

};
#endif // LEPTON_THREAD_H

