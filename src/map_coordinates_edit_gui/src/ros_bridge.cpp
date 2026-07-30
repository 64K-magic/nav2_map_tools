#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <QMetaObject>
#include <memory>
#include <chrono>

// forward
class MainWindowWrapper;

void run_ros_bridge(MainWindowWrapper *w) {
    // create node on this thread
    auto node = std::make_shared<rclcpp::Node>("map_gui_bridge");

    // lambda helpers to invoke QWidget slots via queued connection
    auto call_gps = [w](double lat, double lon){
        QMetaObject::invokeMethod(reinterpret_cast<QObject*>(w), "onGpsUpdate", Qt::QueuedConnection,
                                  Q_ARG(double, lat), Q_ARG(double, lon));
    };

    // subscriptions
    auto gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, [call_gps](const sensor_msgs::msg::NavSatFix::SharedPtr msg){
            call_gps(msg->latitude, msg->longitude);
        }
    );

    // publisher for prohibition areas
    auto prohibition_pub = node->create_publisher<geometry_msgs::msg::PoseArray>("prohibition_areas", 10);

    // Store publisher in a way that can be accessed (simplified for this example)
    // In a real implementation, you'd pass this back or use a callback

    rclcpp::Rate rate(50);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}
