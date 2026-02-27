#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "unity_stream_parser.h"
#include "rgb_camera_parser.h"
#include "depth_camera_parser.h"
#include "fisheye_camera_parser.h"
#include "imu_parser.h"
#include "true_state_parser.h"
#include "unity_command_stream.h"


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("unity_ros");

  RCLCPP_INFO(node->get_logger(), "Starting TCPStreamReader");
  
  TCPStreamReader stream_reader("127.0.0.1", "9998");
  RCLCPP_INFO(node->get_logger(), "Waiting for connection...");
  stream_reader.WaitConnect();
  RCLCPP_INFO(node->get_logger(), "Got a connection...");

  IMUParser imu_parser;
  UnityCommandStream command_stream("127.0.0.1", "9999");

  std::vector<std::shared_ptr<UnityStreamParser>> stream_parsers(UnityMessageType::MESSAGE_TYPE_COUNT);

  stream_parsers[UnityMessageType::UNITY_STATE] = std::make_shared<TrueStateParser>();
  stream_parsers[UnityMessageType::UNITY_IMU] = std::make_shared<IMUParser>();
  stream_parsers[UnityMessageType::UNITY_CAMERA] = std::make_shared<RGBCameraParser>();
  stream_parsers[UnityMessageType::UNITY_DEPTH] = std::make_shared<DepthCameraParser>();
  stream_parsers[UnityMessageType::UNITY_FISHEYE] = std::make_shared<FisheyeCameraParser>();
  
  while (stream_reader.Good() && rclcpp::ok()) {    
    uint32_t magic = stream_reader.ReadUInt();


    if(magic == 0xDEADC0DE) {
      double ros_time = node->now().seconds();
      UnityHeader header;
      header.type = static_cast<UnityMessageType>(stream_reader.ReadUInt());
      uint64_t timestamp_raw = stream_reader.ReadUInt64();
      header.timestamp = static_cast<double>(timestamp_raw) * 1e-7;
      header.name = stream_reader.ReadString();
      
      if(header.type < UnityMessageType::MESSAGE_TYPE_COUNT) {
        stream_parsers[header.type]->ParseMessage(header, stream_reader);
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Stream corrupted, could not parse unity message");
    }

    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
