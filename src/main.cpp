#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "flir_lepton/LeptonThread.h"
#define RAINBOW 1
#define GRAYSCALE 2
#define IRONBLACK 3
#define LEPTON_2 2
#define LEPTON_3 3

int main( int argc, char **argv )
{
  printf("driver starting...\n");
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("flir_lepton");

  // parameters from launch or YAML file (with defaults)
  int typeColormap = node->declare_parameter("typeColormap", IRONBLACK);
  int typeLepton = node->declare_parameter("typeLepton", LEPTON_3);
  int spiSpeed = node->declare_parameter("spiSpeed", 20);
  int rangeMin = node->declare_parameter("rangeMin", -1);
  int rangeMax = node->declare_parameter("rangeMax", -1);
  int loglevel = node->declare_parameter("loglevel", 0);
  int autoScale = node->declare_parameter("autoScale", 1);
  std::string topicName = node->declare_parameter("topicName", std::string("thermal_image"));

  // log the configuration
  RCLCPP_INFO(node->get_logger(), "Flir Lepton typeColormap: %d", typeColormap);
  RCLCPP_INFO(node->get_logger(), "Flir Lepton typeLepton: %d, SpiSpeed: %d", typeColormap, spiSpeed);
  RCLCPP_INFO(node->get_logger(), "Flir Lepton rangeMin: %d, rangeMax: %d", rangeMin, rangeMax);

  // ROS2 publisher for thermal image
  auto imagePublisher = node->create_publisher<sensor_msgs::msg::Image>(topicName, 10);

  // create and configure the Leption SPI capture thread
  auto lepton = std::make_shared<LeptonThread>(node);
  lepton->setLogLevel(loglevel);
  lepton->useColormap(typeColormap);
  lepton->useLepton(typeLepton);
  lepton->useSpiSpeedMhz(spiSpeed);
  lepton->setAutomaticScalingRange(autoScale);
  lepton->setPublisher(imagePublisher);
  lepton->setNode(node);
  if (0 <= rangeMin) lepton->useRangeMinValue(rangeMin);
  if (0 <= rangeMax) lepton->useRangeMaxValue(rangeMax);


  // Register ROS2 service to trigger Flat Field Correction (FFC)
  auto service = node->create_service<std_srvs::srv::Empty>(
    "performFCC",
    std::bind(&LeptonThread::performFFC, lepton.get(), std::placeholders::_1, std::placeholders::_2)
  );


  // Start SPI
  printf("Starting run...\n");
  lepton->run();
  printf("run() finished...\n");

  // Keep the node alive
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
  
}
