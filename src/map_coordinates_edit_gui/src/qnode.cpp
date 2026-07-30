#include "map_coordinates_edit_gui/qnode.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QEventLoop>
#include <QFileInfo>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

QNode::QNode(QObject * parent)
: QObject(parent)
{
  node_ = std::make_shared<rclcpp::Node>("map_coordinates_edit_gui");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  setup_subscriptions();
  setFromLlService(from_ll_service_);
  setToLlService(to_ll_service_);
  setDatumService(set_datum_service_);
}

QNode::~QNode()
{
  if (navsat_process_) {
    if (navsat_process_->state() != QProcess::NotRunning) {
      navsat_process_->terminate();
      if (!navsat_process_->waitForFinished(3000)) {
        navsat_process_->kill();
        navsat_process_->waitForFinished(1000);
      }
    }
    delete navsat_process_;
    navsat_process_ = nullptr;
  }
  if (exec_) {
    exec_->cancel();
  }
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
}

void QNode::start()
{
  exec_thread_ = std::thread([this]() {
      exec_->spin();
    });
}

void QNode::setup_subscriptions()
{
  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", 10,
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
      emit gpsUpdated(msg->latitude, msg->longitude);
    });

  prohibition_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "prohibition_areas", 10);
  // navsat_transform needs at least one /odometry/filtered before SetDatum
  // can make transform_good_ (otherwise /fromLL returns 0,0,0).
  odom_stub_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10);
  keepout_refresh_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "keepout_refresh", 10);
}

void QNode::publishProhibitionAreas(const geometry_msgs::msg::PoseArray & areas)
{
  prohibition_pub_->publish(areas);
}

void QNode::publishKeepoutMapSwitch(const std::string & map_name)
{
  if (!keepout_refresh_pub_ || map_name.empty()) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = "map:" + map_name;
  keepout_refresh_pub_->publish(msg);
  emit statusMessage(QString("Published keepout_refresh: %1").arg(QString::fromStdString(msg.data)));
}

void QNode::setFromLlService(const std::string & name)
{
  from_ll_service_ = name.empty() ? "/fromLL" : name;
  from_ll_client_ = node_->create_client<robot_localization::srv::FromLL>(from_ll_service_);
}

void QNode::setToLlService(const std::string & name)
{
  to_ll_service_ = name.empty() ? "/toLL" : name;
  to_ll_client_ = node_->create_client<robot_localization::srv::ToLL>(to_ll_service_);
}

void QNode::setDatumService(const std::string & name)
{
  set_datum_service_ = name.empty() ? "/datum" : name;
  set_datum_client_ = node_->create_client<robot_localization::srv::SetDatum>(set_datum_service_);
}

bool QNode::navsatServicesReady(std::chrono::milliseconds timeout) const
{
  if (!from_ll_client_ || !set_datum_client_) {
    return false;
  }
  const bool from_ok = from_ll_client_->wait_for_service(timeout);
  const bool datum_ok = set_datum_client_->wait_for_service(timeout);
  return from_ok && datum_ok;
}

QString QNode::standaloneParamsPath() const
{
  // Prefer sibling of executable: ../config/navsat_transform_standalone.yaml
  const QString exe = QCoreApplication::applicationDirPath();
  const QString local = QDir(exe).absoluteFilePath("../config/navsat_transform_standalone.yaml");
  if (QFileInfo::exists(local)) {
    return local;
  }
  // Fallback: source-tree style path under home workspace
  const QString home = QDir::homePath() +
    "/gps_filter_ws/src/map_coordinates_edit_gui/config/navsat_transform_standalone.yaml";
  if (QFileInfo::exists(home)) {
    return home;
  }
  return local;
}

bool QNode::startNavsatTransformProcess(std::string & error)
{
  if (navsat_process_ && navsat_process_->state() != QProcess::NotRunning) {
    return true;
  }

  const QString params = standaloneParamsPath();
  if (!QFileInfo::exists(params)) {
    error = "Cannot find navsat_transform_standalone.yaml at " + params.toStdString();
    return false;
  }

  if (!navsat_process_) {
    navsat_process_ = new QProcess(this);
    navsat_process_->setProcessChannelMode(QProcess::MergedChannels);
    connect(navsat_process_, &QProcess::readyReadStandardOutput, this, [this]() {
        const QByteArray out = navsat_process_->readAllStandardOutput();
        qDebug().noquote() << "[navsat_transform]" << out;
      });
  }

  QStringList args;
  args << "run" << "robot_localization" << "navsat_transform_node"
       << "--ros-args"
       << "-r" << "__node:=navsat_transform"
       << "--params-file" << params;

  emit statusMessage(QString("Starting navsat_transform_node..."));
  qDebug() << "Launching: ros2" << args;
  navsat_process_->start("ros2", args);
  if (!navsat_process_->waitForStarted(5000)) {
    error = "Failed to start ros2 run robot_localization navsat_transform_node: " +
      navsat_process_->errorString().toStdString();
    return false;
  }
  return true;
}

bool QNode::ensureNavsatTransform(std::string & error, std::chrono::milliseconds wait)
{
  // Refresh clients in case service names changed
  setFromLlService(from_ll_service_);
  setToLlService(to_ll_service_);
  setDatumService(set_datum_service_);

  if (navsatServicesReady(200ms)) {
    emit statusMessage("navsat_transform services already available");
    return true;
  }

  emit statusMessage("navsat_transform /fromLL not found — starting node");
  if (!startNavsatTransformProcess(error)) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + wait;
  while (std::chrono::steady_clock::now() < deadline) {
    if (navsat_process_ && navsat_process_->state() == QProcess::NotRunning) {
      error = "navsat_transform_node exited early. Output:\n" +
        QString::fromLocal8Bit(navsat_process_->readAllStandardOutput()).toStdString();
      return false;
    }
    if (navsatServicesReady(500ms)) {
      emit statusMessage("navsat_transform ready (/fromLL, /datum)");
      return true;
    }
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
  }

  error = "Timed out waiting for /fromLL and /datum after starting navsat_transform_node";
  return false;
}

void QNode::publishStubOdometry()
{
  if (!odom_stub_pub_) {
    return;
  }
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = node_->now();
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.w = 1.0;
  odom_stub_pub_->publish(odom);
}

bool QNode::fromLL(
  double lat, double lon, double alt,
  geometry_msgs::msg::Point & map_point, std::string & error)
{
  if (!from_ll_client_) {
    error = "fromLL client not initialized";
    return false;
  }
  if (!from_ll_client_->wait_for_service(2s)) {
    error = "fromLL service not available (" + from_ll_service_ +
      "). Start navsat_transform_node and wait until transform is ready.";
    return false;
  }

  auto req = std::make_shared<robot_localization::srv::FromLL::Request>();
  req->ll_point.latitude = lat;
  req->ll_point.longitude = lon;
  req->ll_point.altitude = alt;

  auto future = from_ll_client_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    error = "fromLL call timed out";
    return false;
  }
  auto resp = future.get();
  if (!resp) {
    error = "fromLL returned null (transform may not be ready)";
    return false;
  }
  map_point = resp->map_point;
  return true;
}

bool QNode::toLL(
  double x, double y, double z,
  double & lat, double & lon, double & alt, std::string & error)
{
  if (!to_ll_client_) {
    error = "toLL client not initialized";
    return false;
  }
  if (!to_ll_client_->wait_for_service(2s)) {
    error = "toLL service not available (" + to_ll_service_ + ")";
    return false;
  }

  auto req = std::make_shared<robot_localization::srv::ToLL::Request>();
  req->map_point.x = x;
  req->map_point.y = y;
  req->map_point.z = z;

  auto future = to_ll_client_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    error = "toLL call timed out";
    return false;
  }
  auto resp = future.get();
  if (!resp) {
    error = "toLL returned null";
    return false;
  }
  lat = resp->ll_point.latitude;
  lon = resp->ll_point.longitude;
  alt = resp->ll_point.altitude;
  return true;
}

bool QNode::setDatum(double lat, double lon, double yaw_rad, std::string & error)
{
  if (!set_datum_client_) {
    error = "SetDatum client not initialized";
    return false;
  }
  if (!set_datum_client_->wait_for_service(2s)) {
    error = "SetDatum service not available (" + set_datum_service_ + ")";
    return false;
  }

  // Prime odometry BEFORE SetDatum so navsat sets has_transform_odom_.
  for (int i = 0; i < 5; ++i) {
    publishStubOdometry();
    std::this_thread::sleep_for(50ms);
  }

  auto req = std::make_shared<robot_localization::srv::SetDatum::Request>();
  req->geo_pose.position.latitude = lat;
  req->geo_pose.position.longitude = lon;
  req->geo_pose.position.altitude = 0.0;
  const double cy = std::cos(yaw_rad * 0.5);
  const double sy = std::sin(yaw_rad * 0.5);
  req->geo_pose.orientation.x = 0.0;
  req->geo_pose.orientation.y = 0.0;
  req->geo_pose.orientation.z = sy;
  req->geo_pose.orientation.w = cy;

  auto future = set_datum_client_->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    error = "SetDatum call timed out";
    return false;
  }

  // Best-effort: make /fromLL usable for stacks that need it. Save/Load use local ENU.
  std::string ready_err;
  if (waitUntilFromLlReady(lat, lon, ready_err)) {
    emit statusMessage("SetDatum OK; /fromLL transform ready");
  } else {
    emit statusMessage(QString::fromStdString(ready_err));
    qWarning() << QString::fromStdString(ready_err);
  }
  return true;
}

bool QNode::waitUntilFromLlReady(
  double datum_lat, double datum_lon, std::string & error,
  std::chrono::milliseconds wait)
{
  const auto deadline = std::chrono::steady_clock::now() + wait;
  // ~50 m east of datum — must not map to (0,0) if transform is good.
  const double probe_lat = datum_lat;
  const double probe_lon = datum_lon + 0.0005;

  while (std::chrono::steady_clock::now() < deadline) {
    publishStubOdometry();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);

    geometry_msgs::msg::Point p0, p1;
    std::string err;
    if (fromLL(datum_lat, datum_lon, 0.0, p0, err) &&
        fromLL(probe_lat, probe_lon, 0.0, p1, err))
    {
      const double dist = std::hypot(p1.x - p0.x, p1.y - p0.y);
      // Expect roughly 40–60 m for 0.0005 deg lon near mid-latitudes.
      if (dist > 5.0) {
        return true;
      }
    }
    std::this_thread::sleep_for(100ms);
  }

  error =
    "navsat_transform /fromLL still returns zeros after SetDatum.\n"
    "The node needs /odometry/filtered (stub published) and a short settle time.\n"
    "GUI Save/Load will use local ENU from Origin Lat/Lon instead.";
  return false;
}
