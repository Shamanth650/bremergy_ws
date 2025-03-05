#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "driverless_intro/msg/cone.hpp"
#include "driverless_intro/msg/track.hpp"
#include "std_msgs/msg/header.hpp"
#include <cmath>
#include <limits>

using std::placeholders::_1;

class NearestConeFinder : public rclcpp::Node
{
public:
    NearestConeFinder() : Node("nearest_cone_finder")
    {
        cat_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/cat_position", 10, std::bind(&NearestConeFinder::catPositionCallback, this, _1));

        cones_sub_ = this->create_subscription<driverless_intro::msg::Track>(
            "/cones", 10, std::bind(&NearestConeFinder::conesCallback, this, _1));

        target_cone_pub_ = this->create_publisher<driverless_intro::msg::Cone>("/target_cone", 10);
    }

private:
    void catPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        cat_position_ = *msg;
        cat_position_received_ = true;
        findNearestCone();
    }

    void conesCallback(const driverless_intro::msg::Track::SharedPtr msg)
    {
        cones_ = msg->cones;
        findNearestCone();
    }

    void findNearestCone()
    {
        if (!cat_position_received_ || cones_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot find nearest cone: missing data.");
            return;
        }

        double min_distance = std::numeric_limits<double>::max();
        driverless_intro::msg::Cone nearest_cone;

        for (const auto & cone : cones_) {
            double distance = std::hypot(cone.position.x - cat_position_.x, 
                                         cone.position.y - cat_position_.y);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_cone = cone;
            }
        }

        if (min_distance == std::numeric_limits<double>::max()) {
            RCLCPP_WARN(this->get_logger(), "No valid cones found.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Nearest cone at x=%.2f, y=%.2f", nearest_cone.position.x, nearest_cone.position.y);
        target_cone_pub_->publish(nearest_cone);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cat_position_sub_;
    rclcpp::Subscription<driverless_intro::msg::Track>::SharedPtr cones_sub_;
    rclcpp::Publisher<driverless_intro::msg::Cone>::SharedPtr target_cone_pub_;

    geometry_msgs::msg::Point cat_position_;
    std::vector<driverless_intro::msg::Cone> cones_;
    bool cat_position_received_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NearestConeFinder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

