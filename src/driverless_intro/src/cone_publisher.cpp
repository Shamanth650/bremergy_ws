#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "driverless_intro/msg/cone.hpp"
#include "driverless_intro/msg/track.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

class ConePublisher : public rclcpp::Node
{
public:
  ConePublisher(const std::string & csv_file_path)
  : Node("cone_publisher")
  {
    publisher_ = this->create_publisher<driverless_intro::msg::Track>("/cones", 10);

    if (!read_csv(csv_file_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read CSV file: %s", csv_file_path.c_str());
      rclcpp::shutdown();
      return;
    }

    
    timer_ = this->create_wall_timer(500ms, std::bind(&ConePublisher::publish_track, this));
  }

private:
  
  uint8_t color_to_type(const std::string & color)
  {
    if (color == "blue")         return driverless_intro::msg::Cone::BLUE;
    if (color == "yellow")       return driverless_intro::msg::Cone::YELLOW;
    if (color == "orange_big")   return driverless_intro::msg::Cone::ORANGE_BIG;
    if (color == "orange_small") return driverless_intro::msg::Cone::ORANGE_SMALL;
    return driverless_intro::msg::Cone::UNKNOWN;
  }

  
  bool read_csv(const std::string & csv_file_path)
  {
    std::ifstream file(csv_file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open CSV file: %s", csv_file_path.c_str());
      return false;
    }

    
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
     
      if (line.empty()) {
        continue;
      }

      
      std::stringstream line_ss(line);

      std::string color_str;
      std::string pose_str; 
      std::string name_str;

      
      if (!std::getline(line_ss, color_str, ',')) {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid line (missing color): %s", line.c_str());
        continue;
      }
      if (!std::getline(line_ss, pose_str, ',')) {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid line (missing pose): %s", line.c_str());
        continue;
      }
      if (!std::getline(line_ss, name_str, ',')) {
        RCLCPP_WARN(this->get_logger(), "Skipping invalid line (missing name): %s", line.c_str());
        continue;
      }

      
      double x, y;
      {
        std::stringstream pose_fields(pose_str);
        if (!(pose_fields >> x >> y)) {
          RCLCPP_WARN(this->get_logger(), "Invalid pose format (cannot parse x,y): %s", pose_str.c_str());
          continue;
        }
      }

      // Construct Cone message
      driverless_intro::msg::Cone cone_msg;
      cone_msg.position.x = x;
      cone_msg.position.y = y;
      cone_msg.position.z = 0.0;  
      cone_msg.type = color_to_type(color_str);
      cone_msg.knocked_over = false;

      cones_.push_back(cone_msg);
    }

    file.close();
    return true;
  }

  void publish_track()
  {
    driverless_intro::msg::Track track_msg;
    track_msg.header.stamp = this->now();
    track_msg.header.frame_id = "map";
    track_msg.cones = cones_;

    publisher_->publish(track_msg);
    RCLCPP_INFO(this->get_logger(), "Published Track message with %zu cones.", cones_.size());
  }

  rclcpp::Publisher<driverless_intro::msg::Track>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<driverless_intro::msg::Cone> cones_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: cone_publisher <path_to_csv_file>" << std::endl;
    return 1;
  }

  std::string csv_file_path = argv[1];
  auto node = std::make_shared<ConePublisher>(csv_file_path);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

