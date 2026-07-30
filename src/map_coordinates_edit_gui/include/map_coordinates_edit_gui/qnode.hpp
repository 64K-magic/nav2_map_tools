#ifndef MAP_COORDINATES_EDIT_GUI_QNODE_HPP_
#define MAP_COORDINATES_EDIT_GUI_QNODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include <QObject>
#include <QProcess>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <robot_localization/srv/set_datum.hpp>
#include <robot_localization/srv/to_ll.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

class QNode : public QObject {
  Q_OBJECT

public:
  explicit QNode(QObject * parent = nullptr);
  ~QNode();

  void start();
  void publishProhibitionAreas(const geometry_msgs::msg::PoseArray & areas);

  /**
   * Ask filter_keepout to load keepouts for map_name.
   * Publishes to keepout_refresh as "map:<map_name>".
   */
  void publishKeepoutMapSwitch(const std::string & map_name);

  void setFromLlService(const std::string & name);
  void setToLlService(const std::string & name);
  void setDatumService(const std::string & name);

  /** True if /fromLL (and optionally /datum) are already advertised. */
  bool navsatServicesReady(std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) const;

  /**
   * Ensure navsat_transform is running with /fromLL and /datum available.
   * If missing, starts robot_localization::navsat_transform_node (wait_for_datum).
   */
  bool ensureNavsatTransform(std::string & error, std::chrono::milliseconds wait = std::chrono::seconds(20));

  /**
   * Publish a stub /odometry/filtered at map origin.
   * Required by navsat_transform even with SetDatum (has_transform_odom_).
   */
  void publishStubOdometry();

  /** Convert geographic lat/lon/alt to map-frame meters via /fromLL. */
  bool fromLL(double lat, double lon, double alt, geometry_msgs::msg::Point & map_point, std::string & error);

  /** Convert map-frame meters to lat/lon via /toLL. */
  bool toLL(double x, double y, double z, double & lat, double & lon, double & alt, std::string & error);

  /**
   * Set navsat_transform datum and prime stub odometry so /fromLL becomes valid.
   * Without odometry, FromLL silently returns (0,0,0).
   */
  bool setDatum(double lat, double lon, double yaw_rad, std::string & error);

  /** Probe that FromLL is usable (non-zero for a known offset from datum). */
  bool waitUntilFromLlReady(double datum_lat, double datum_lon, std::string & error,
                            std::chrono::milliseconds wait = std::chrono::seconds(5));

  rclcpp::Node::SharedPtr node() const {return node_;}

signals:
  void gpsUpdated(double lat, double lon);
  void prohibitionAreasUpdated(const geometry_msgs::msg::PoseArray & areas);
  void statusMessage(const QString & msg);

private:
  void setup_subscriptions();
  bool startNavsatTransformProcess(std::string & error);
  QString standaloneParamsPath() const;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread exec_thread_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr prohibition_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_stub_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keepout_refresh_pub_;

  rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;
  rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr to_ll_client_;
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr set_datum_client_;

  std::string from_ll_service_{"/fromLL"};
  std::string to_ll_service_{"/toLL"};
  std::string set_datum_service_{"/datum"};

  QProcess * navsat_process_{nullptr};
};

#endif  // MAP_COORDINATES_EDIT_GUI_QNODE_HPP_
