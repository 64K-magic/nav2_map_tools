#include "map_coordinates_edit_gui/pgm_assist.hpp"

#include "map_coordinates_edit_gui/pgm_io.hpp"
#include "map_coordinates_edit_gui/speed_plot.hpp"

#include <QCheckBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QTextStream>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace
{
constexpr int kZBrush = 900;
constexpr int kZRobot = 700;
constexpr int kZScan = 650;
constexpr int kZPath = 500;
constexpr int kZPathCtrl = 550;
}  // namespace

PgmAssistController::PgmAssistController(QObject * parent)
: QObject(parent)
{
}

PgmAssistController::~PgmAssistController()
{
  clearPgmMap();
}

void PgmAssistController::attach(
  QGraphicsScene * scene, QGraphicsPixmapItem * map_item, QGraphicsView * view)
{
  scene_ = scene;
  map_item_ = map_item;
  if (view_ && view_->viewport()) {
    view_->viewport()->removeEventFilter(this);
  }
  view_ = view;
  if (view_ && view_->viewport()) {
    view_->viewport()->installEventFilter(this);
  }
}

void PgmAssistController::setNode(const rclcpp::Node::SharedPtr & node)
{
  node_ = node;
}

void PgmAssistController::setPgmMap(
  const QImage & editable, const QString & map_path,
  double resolution, double origin_x, double origin_y)
{
  clearPathItems(false);
  clearPersistentPath();
  clearScanItems();
  clearObstacleItems();
  clearRobotItems();
  clearBrushOverlay();

  image_ = editable.convertToFormat(QImage::Format_Grayscale8);
  map_path_ = map_path;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  undo_stack_.clear();
  if (!image_.isNull()) {
    undo_stack_.push_back(image_);
  }
  syncPixmap();
  setToolMode(ToolMode::Idle);
  emit statusMessage(QStringLiteral("PGM 辅助已就绪: %1").arg(QFileInfo(map_path).fileName()));
}

void PgmAssistController::clearPgmMap()
{
  setToolMode(ToolMode::Idle);
  clearPathItems(false);
  clearPersistentPath();
  clearScanItems();
  clearObstacleItems();
  clearRobotItems();
  clearBrushOverlay();
  image_ = QImage();
  map_path_.clear();
  undo_stack_.clear();
  have_robot_pose_ = false;
}

void PgmAssistController::setToolMode(ToolMode mode)
{
  if (tool_mode_ == ToolMode::PathDraw && mode != ToolMode::PathDraw) {
    // keep endpoints until save/clear; just stop preview cursor line
    preview_enabled_ = false;
    updatePathPreview();
  }
  tool_mode_ = mode;
  painting_ = false;
  clearBrushOverlay();
  if (brush_btn_) {
    brush_btn_->setChecked(mode == ToolMode::Brush);
  }
  if (path_btn_) {
    path_btn_->setChecked(mode == ToolMode::PathDraw);
  }
  if (mode == ToolMode::PathDraw) {
    preview_enabled_ = true;
  }
  emit toolModeChanged(mode != ToolMode::Idle);
}

void PgmAssistController::setBrushRadius(int r)
{
  brush_radius_ = std::max(1, r);
  if (brush_slider_ && brush_slider_->value() != brush_radius_) {
    brush_slider_->setValue(brush_radius_);
  }
}

bool PgmAssistController::savePgm(const QString & path, QString & error)
{
  if (image_.isNull()) {
    error = QStringLiteral("No PGM image loaded");
    return false;
  }
  if (!pgm_io::savePGM(path, image_)) {
    error = QStringLiteral("Failed to write PGM: %1").arg(path);
    return false;
  }
  map_path_ = path;
  emit statusMessage(QStringLiteral("已保存 PGM: %1").arg(path));
  return true;
}

bool PgmAssistController::undoBrush()
{
  if (undo_stack_.size() < 2) {
    return false;
  }
  undo_stack_.pop_back();
  image_ = undo_stack_.back();
  syncPixmap();
  emit imageEdited();
  return true;
}

void PgmAssistController::syncPixmap()
{
  if (!map_item_ || image_.isNull()) {
    return;
  }
  map_item_->setPixmap(QPixmap::fromImage(image_));
}

void PgmAssistController::pushUndo()
{
  if (image_.isNull()) {
    return;
  }
  undo_stack_.push_back(image_);
  if (static_cast<int>(undo_stack_.size()) > max_undo_) {
    undo_stack_.erase(undo_stack_.begin());
  }
}

void PgmAssistController::paintAt(const QPointF & scene_pt)
{
  if (image_.isNull()) {
    return;
  }
  const int x = static_cast<int>(scene_pt.x());
  const int y = static_cast<int>(scene_pt.y());
  if (x < 0 || y < 0 || x >= image_.width() || y >= image_.height()) {
    return;
  }
  QPainter p(&image_);
  p.setCompositionMode(QPainter::CompositionMode_Source);
  const QColor c(draw_value_, draw_value_, draw_value_);
  p.setBrush(QBrush(c));
  p.setPen(Qt::NoPen);
  p.drawEllipse(QPoint(x, y), brush_radius_, brush_radius_);
  p.end();
  syncPixmap();
  emit imageEdited();
}

void PgmAssistController::updateBrushOverlay(const QPointF & scene_pt)
{
  if (!scene_ || image_.isNull()) {
    return;
  }
  const int r = brush_radius_;
  const QRectF rect(scene_pt.x() - r, scene_pt.y() - r, 2 * r, 2 * r);
  if (!brush_overlay_) {
    brush_overlay_ = scene_->addEllipse(
      rect, QPen(QColor(80, 80, 80, 200)), QBrush(QColor(120, 120, 120, 80)));
    brush_overlay_->setZValue(kZBrush);
  } else {
    brush_overlay_->setRect(rect);
  }
  brush_overlay_->setVisible(true);
}

void PgmAssistController::clearBrushOverlay()
{
  if (brush_overlay_) {
    brush_overlay_->setVisible(false);
  }
}

void PgmAssistController::clearPathItems(bool /*keep_persistent*/)
{
  if (!scene_) {
    return;
  }
  for (auto * it : end_items_) {
    scene_->removeItem(it);
    delete it;
  }
  end_items_.clear();
  for (auto * it : control_items_) {
    scene_->removeItem(it);
    delete it;
  }
  control_items_.clear();
  end_points_.clear();
  control_points_.clear();
  if (preview_path_) {
    scene_->removeItem(preview_path_);
    delete preview_path_;
    preview_path_ = nullptr;
  }
}

void PgmAssistController::clearPersistentPath()
{
  if (!scene_) {
    return;
  }
  if (persistent_path_) {
    scene_->removeItem(persistent_path_);
    delete persistent_path_;
    persistent_path_ = nullptr;
  }
  for (auto * it : path_point_items_) {
    scene_->removeItem(it);
    delete it;
  }
  path_point_items_.clear();
  for (auto * it : path_arrow_items_) {
    scene_->removeItem(it);
    delete it;
  }
  path_arrow_items_.clear();
  persistent_pts_.clear();
}

void PgmAssistController::addPathEndpoint(const QPointF & p)
{
  if (!scene_) {
    return;
  }
  end_points_.push_back(p);
  auto * end_item = scene_->addEllipse(
    p.x() - 3, p.y() - 3, 6, 6, QPen(QColor(0, 255, 0)), QBrush(QColor(0, 255, 0, 200)));
  end_item->setZValue(kZPathCtrl);
  end_items_.push_back(end_item);
  if (end_points_.size() >= 2) {
    const QPointF mid = (end_points_[end_points_.size() - 2] + end_points_.back()) * 0.5;
    control_points_.push_back(mid);
    auto * cp = scene_->addEllipse(
      mid.x() - 6, mid.y() - 6, 12, 12, QPen(QColor(0, 200, 0)), QBrush(QColor(0, 255, 0, 120)));
    cp->setZValue(kZPathCtrl + 1);
    control_items_.push_back(cp);
  }
  preview_enabled_ = true;
  updatePathPreview();
}

std::vector<QPointF> PgmAssistController::samplePath(int samples_per_segment) const
{
  std::vector<QPointF> out;
  const int n = static_cast<int>(end_points_.size());
  if (n < 2) {
    return out;
  }
  for (int i = 0; i < n - 1; ++i) {
    const QPointF p0 = end_points_[i];
    const QPointF p1 = end_points_[i + 1];
    const QPointF c = control_points_[i];
    for (int j = 0; j <= samples_per_segment; ++j) {
      const double t = double(j) / double(samples_per_segment);
      const double u = 1.0 - t;
      out.push_back(u * u * p0 + 2 * u * t * c + t * t * p1);
    }
  }
  return out;
}

void PgmAssistController::updatePathPreview(const QPointF * cursor)
{
  if (!scene_) {
    return;
  }
  if (preview_path_) {
    scene_->removeItem(preview_path_);
    delete preview_path_;
    preview_path_ = nullptr;
  }
  auto pts = samplePath(80);
  if (preview_enabled_ && cursor && !end_points_.empty()) {
    if (pts.empty()) {
      pts.push_back(end_points_.back());
    }
    pts.push_back(*cursor);
  }
  if (pts.size() < 2) {
    return;
  }
  QPainterPath path;
  path.moveTo(pts[0]);
  for (size_t i = 1; i < pts.size(); ++i) {
    path.lineTo(pts[i]);
  }
  preview_path_ = scene_->addPath(path, QPen(QColor(30, 120, 255), 1, Qt::DashLine));
  preview_path_->setZValue(kZPath);
}

void PgmAssistController::drawPersistentPath(const std::vector<QPointF> & pts)
{
  clearPersistentPath();
  persistent_pts_ = pts;
  redrawPersistentPath();
}

void PgmAssistController::redrawPersistentPath()
{
  if (!scene_ || persistent_pts_.empty()) {
    return;
  }
  if (persistent_path_) {
    scene_->removeItem(persistent_path_);
    delete persistent_path_;
    persistent_path_ = nullptr;
  }
  for (auto * it : path_point_items_) {
    scene_->removeItem(it);
    delete it;
  }
  path_point_items_.clear();
  for (auto * it : path_arrow_items_) {
    scene_->removeItem(it);
    delete it;
  }
  path_arrow_items_.clear();

  if (show_path_) {
    QPainterPath path;
    path.moveTo(persistent_pts_[0]);
    for (size_t i = 1; i < persistent_pts_.size(); ++i) {
      path.lineTo(persistent_pts_[i]);
    }
    persistent_path_ = scene_->addPath(path, QPen(QColor(200, 20, 20), 1));
    persistent_path_->setZValue(kZPath);
  }
  if (show_points_) {
    for (size_t i = 0; i < persistent_pts_.size(); i += std::max(1, point_density_)) {
      const QPointF & pt = persistent_pts_[i];
      auto * item = scene_->addEllipse(
        pt.x() - 2, pt.y() - 2, 4, 4, QPen(QColor(20, 20, 200)), QBrush(QColor(20, 20, 200, 150)));
      item->setZValue(kZPath - 50);
      path_point_items_.push_back(item);
    }
  }
  if (show_arrows_) {
    for (size_t i = 0; i < persistent_pts_.size(); i += std::max(1, arrow_density_)) {
      const QPointF & pt = persistent_pts_[i];
      double yaw = 0.0;
      if (i + 1 < persistent_pts_.size()) {
        const QPointF d = persistent_pts_[i + 1] - pt;
        yaw = std::atan2(d.y(), d.x());
      } else if (i > 0) {
        const QPointF d = pt - persistent_pts_[i - 1];
        yaw = std::atan2(d.y(), d.x());
      }
      const double arrow_len = 8.0;
      const double arrow_w = 4.0;
      QPolygonF poly;
      poly << (pt + QPointF(std::cos(yaw) * arrow_len, std::sin(yaw) * arrow_len))
           << (pt + QPointF(std::cos(yaw + M_PI / 2) * arrow_w, std::sin(yaw + M_PI / 2) * arrow_w))
           << (pt + QPointF(std::cos(yaw - M_PI / 2) * arrow_w, std::sin(yaw - M_PI / 2) * arrow_w));
      auto * item = scene_->addPolygon(poly, QPen(QColor(255, 0, 0)), QBrush(QColor(255, 0, 0, 200)));
      item->setZValue(kZPath - 40);
      path_arrow_items_.push_back(item);
    }
  }
}

void PgmAssistController::setShowPath(bool on)
{
  show_path_ = on;
  redrawPersistentPath();
}
void PgmAssistController::setShowPoints(bool on)
{
  show_points_ = on;
  redrawPersistentPath();
}
void PgmAssistController::setShowArrows(bool on)
{
  show_arrows_ = on;
  redrawPersistentPath();
}
void PgmAssistController::setPointDensity(int n)
{
  point_density_ = std::max(1, n);
  redrawPersistentPath();
}
void PgmAssistController::setArrowDensity(int n)
{
  arrow_density_ = std::max(1, n);
  redrawPersistentPath();
}

QPointF PgmAssistController::worldToScene(double x, double y) const
{
  const double h = image_.isNull() ? 0.0 : static_cast<double>(image_.height());
  return QPointF(
    (x - origin_x_) / resolution_,
    h - 1.0 - (y - origin_y_) / resolution_);
}

void PgmAssistController::sceneToWorld(const QPointF & scene_pt, double & x, double & y) const
{
  const double h = image_.isNull() ? 0.0 : static_cast<double>(image_.height());
  x = origin_x_ + scene_pt.x() * resolution_;
  y = origin_y_ + (h - 1.0 - scene_pt.y()) * resolution_;
}

bool PgmAssistController::loadYamlBesideMap(
  double & resolution, double & origin_x, double & origin_y) const
{
  resolution = resolution_;
  origin_x = origin_x_;
  origin_y = origin_y_;
  if (map_path_.isEmpty()) {
    return false;
  }
  QFileInfo fi(map_path_);
  QString yaml = fi.absoluteDir().absoluteFilePath(fi.baseName() + QStringLiteral(".yaml"));
  if (!QFile::exists(yaml)) {
    yaml = fi.absoluteDir().absoluteFilePath(fi.baseName() + QStringLiteral(".yml"));
  }
  if (!QFile::exists(yaml)) {
    return false;
  }
  QFile f(yaml);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }
  QTextStream in(&f);
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line.startsWith(QStringLiteral("resolution:"))) {
      resolution = line.mid(11).trimmed().toDouble();
    } else if (line.startsWith(QStringLiteral("origin:"))) {
      const int lb = line.indexOf('[');
      const int rb = line.indexOf(']');
      if (lb >= 0 && rb > lb) {
        const QStringList nums = line.mid(lb + 1, rb - lb - 1).split(',');
        if (nums.size() >= 2) {
          origin_x = nums[0].trimmed().toDouble();
          origin_y = nums[1].trimmed().toDouble();
        }
      }
    }
  }
  return true;
}

std::vector<std::tuple<double, double, double>> PgmAssistController::resamplePath(
  const std::vector<std::tuple<double, double, double>> & path, double target_distance)
{
  std::vector<std::tuple<double, double, double>> resampled;
  if (path.empty()) {
    return resampled;
  }
  resampled.push_back(path[0]);
  for (size_t i = 1; i < path.size(); ++i) {
    double x1, y1, yaw1, x2, y2, yaw2;
    std::tie(x1, y1, yaw1) = path[i - 1];
    std::tie(x2, y2, yaw2) = path[i];
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist > 0.0) {
      const int num_points = static_cast<int>(std::floor(dist / target_distance));
      for (int j = 1; j <= num_points; ++j) {
        const double t = j * target_distance / dist;
        resampled.emplace_back(x1 + t * dx, y1 + t * dy, yaw1 + t * (yaw2 - yaw1));
      }
    }
  }
  resampled.push_back(path.back());
  return resampled;
}

bool PgmAssistController::savePath(const QString & path, QString & error)
{
  auto pts = samplePath(10);
  if (pts.size() < 2) {
    error = QStringLiteral("Need at least 2 path endpoints");
    return false;
  }
  double resolution = resolution_;
  double ox = origin_x_;
  double oy = origin_y_;
  loadYamlBesideMap(resolution, ox, oy);

  std::vector<std::tuple<double, double, double>> out;
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const int px = static_cast<int>(std::round(pts[i].x()));
    const int py = static_cast<int>(std::round(pts[i].y()));
    const int px2 = static_cast<int>(std::round(pts[i + 1].x()));
    const int py2 = static_cast<int>(std::round(pts[i + 1].y()));
    const double h = static_cast<double>(image_.height());
    const double wx = ox + px * resolution;
    const double wy = oy + (h - 1.0 - py) * resolution;
    const double wx2 = ox + px2 * resolution;
    const double wy2 = oy + (h - 1.0 - py2) * resolution;
    out.emplace_back(wx, wy, std::atan2(wy2 - wy, wx2 - wx));
  }
  if (!out.empty()) {
    out.push_back(out.back());
  }
  const auto resampled = resamplePath(out, 0.05);

  QFile of(path);
  if (!of.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = QStringLiteral("Cannot open path file for write");
    return false;
  }
  QTextStream outStream(&of);
  for (const auto & tup : resampled) {
    double x, y, yaw;
    std::tie(x, y, yaw) = tup;
    outStream << QString::number(x, 'f', 6) << ' ' << QString::number(y, 'f', 6) << ' '
              << QString::number(yaw, 'f', 6) << '\n';
  }
  outStream << "EOP\n";
  drawPersistentPath(pts);
  emit statusMessage(QStringLiteral("已保存路径: %1").arg(path));
  return true;
}

bool PgmAssistController::openPath(const QString & path, QString & error)
{
  double resolution = resolution_;
  double ox = origin_x_;
  double oy = origin_y_;
  loadYamlBesideMap(resolution, ox, oy);
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
    error = QStringLiteral("Cannot open path file");
    return false;
  }
  QTextStream in(&f);
  std::vector<QPointF> pts;
  const double h = static_cast<double>(image_.height());
  while (!in.atEnd()) {
    const QString line = in.readLine().trimmed();
    if (line == QLatin1String("EOP")) {
      break;
    }
    const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
    if (parts.size() >= 2) {
      const double x = parts[0].toDouble();
      const double y = parts[1].toDouble();
      pts.emplace_back((x - ox) / resolution, h - 1.0 - (y - oy) / resolution);
    }
  }
  if (pts.empty()) {
    error = QStringLiteral("No path points in file");
    return false;
  }
  drawPersistentPath(pts);
  emit statusMessage(QStringLiteral("已打开路径: %1 (%2 pts)").arg(path).arg(pts.size()));
  return true;
}

void PgmAssistController::clearScanItems()
{
  if (!scene_) {
    return;
  }
  for (auto * it : scan_items_) {
    scene_->removeItem(it);
    delete it;
  }
  scan_items_.clear();
}

void PgmAssistController::clearObstacleItems()
{
  if (!scene_) {
    return;
  }
  for (auto * it : obstacle_items_) {
    scene_->removeItem(it);
    delete it;
  }
  obstacle_items_.clear();
}

void PgmAssistController::clearRobotItems()
{
  if (!scene_) {
    return;
  }
  if (robot_item_) {
    scene_->removeItem(robot_item_);
    delete robot_item_;
    robot_item_ = nullptr;
  }
  if (robot_arrow_) {
    scene_->removeItem(robot_arrow_);
    delete robot_arrow_;
    robot_arrow_ = nullptr;
  }
}

void PgmAssistController::updateRobotPose(double x, double y, double yaw)
{
  if (!scene_ || image_.isNull()) {
    return;
  }
  const QPointF p = worldToScene(x, y);
  if (!robot_item_) {
    robot_item_ = scene_->addEllipse(
      p.x() - 5, p.y() - 5, 10, 10, QPen(QColor(255, 0, 0)), QBrush(QColor(255, 0, 0, 200)));
    robot_item_->setZValue(kZRobot);
  } else {
    robot_item_->setRect(p.x() - 5, p.y() - 5, 10, 10);
  }
  const double arrow_len = 15.0;
  QPolygonF poly;
  poly << (p + QPointF(std::cos(yaw) * arrow_len, std::sin(yaw) * arrow_len))
       << (p + QPointF(std::cos(yaw + M_PI / 2) * 5, std::sin(yaw + M_PI / 2) * 5))
       << (p + QPointF(std::cos(yaw - M_PI / 2) * 5, std::sin(yaw - M_PI / 2) * 5));
  if (!robot_arrow_) {
    robot_arrow_ = scene_->addPolygon(poly, QPen(QColor(255, 0, 0)), QBrush(QColor(255, 0, 0, 200)));
    robot_arrow_->setZValue(kZRobot + 10);
  } else {
    robot_arrow_->setPolygon(poly);
  }
}

void PgmAssistController::updateScanPoints(const std::vector<QPointF> & world_pts)
{
  clearScanItems();
  if (!scene_ || image_.isNull()) {
    return;
  }
  for (const auto & w : world_pts) {
    const QPointF p = worldToScene(w.x(), w.y());
    auto * item = scene_->addEllipse(
      p.x() - 1, p.y() - 1, 2, 2, QPen(QColor(0, 255, 0)), QBrush(QColor(0, 255, 0, 150)));
    item->setZValue(kZScan);
    scan_items_.push_back(item);
  }
}

void PgmAssistController::updateObstaclePoints(const std::vector<QPointF> & world_pts)
{
  clearObstacleItems();
  if (!scene_ || image_.isNull()) {
    return;
  }
  for (const auto & w : world_pts) {
    const QPointF p = worldToScene(w.x(), w.y());
    auto * item = scene_->addEllipse(
      p.x() - 2, p.y() - 2, 4, 4, QPen(QColor(255, 255, 0)), QBrush(QColor(255, 255, 0, 150)));
    item->setZValue(kZScan);
    obstacle_items_.push_back(item);
  }
}

void PgmAssistController::onPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  tf2::Quaternion q(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll = 0, pitch = 0, yaw = 0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  robot_x_ = x;
  robot_y_ = y;
  robot_yaw_ = yaw;
  have_robot_pose_ = true;
  // Qt GUI update from ROS callback thread
  QMetaObject::invokeMethod(this, [this, x, y, yaw]() { updateRobotPose(x, y, yaw); },
                            Qt::QueuedConnection);
}

void PgmAssistController::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!have_robot_pose_) {
    return;
  }
  std::vector<QPointF> points;
  double angle = msg->angle_min;
  for (float r : msg->ranges) {
    if (r > msg->range_min && r < msg->range_max) {
      const double lx = r * std::cos(angle);
      const double ly = r * std::sin(angle);
      const double wx = robot_x_ + lx * std::cos(robot_yaw_) - ly * std::sin(robot_yaw_);
      const double wy = robot_y_ + lx * std::sin(robot_yaw_) + ly * std::cos(robot_yaw_);
      points.emplace_back(wx, wy);
    }
    angle += msg->angle_increment;
  }
  QMetaObject::invokeMethod(this, [this, points]() { updateScanPoints(points); },
                            Qt::QueuedConnection);
}

void PgmAssistController::onObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!have_robot_pose_) {
    return;
  }
  std::vector<QPointF> points;
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const double wx = robot_x_ + (*iter_x) * std::cos(robot_yaw_) -
                        (*iter_y) * std::sin(robot_yaw_);
      const double wy = robot_y_ + (*iter_x) * std::sin(robot_yaw_) +
                        (*iter_y) * std::cos(robot_yaw_);
      points.emplace_back(wx, wy);
    }
  } catch (...) {
    return;
  }
  QMetaObject::invokeMethod(this, [this, points]() { updateObstaclePoints(points); },
                            Qt::QueuedConnection);
}

void PgmAssistController::onSpeed(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double t = node_ ? node_->get_clock()->now().seconds() : 0.0;
  const double lx = msg->linear.x;
  const double ly = msg->linear.y;
  const double az = msg->angular.z;
  QMetaObject::invokeMethod(
    this,
    [this, t, lx, ly, az]() {
      if (speed_plot_) {
        speed_plot_->addData(t, lx, ly, az);
      }
    },
    Qt::QueuedConnection);
}

void PgmAssistController::subscribePose(const std::string & topic)
{
  if (!node_) {
    return;
  }
  unsubscribePose();
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic, rclcpp::QoS(10).best_effort(),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { onPose(msg); });
  pose_subscribed_ = true;
}

void PgmAssistController::subscribeScan(const std::string & topic)
{
  if (!node_) {
    return;
  }
  unsubscribeScan();
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { onScan(msg); });
  scan_subscribed_ = true;
}

void PgmAssistController::subscribeObstacles(const std::string & topic)
{
  if (!node_) {
    return;
  }
  unsubscribeObstacles();
  obstacles_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { onObstacles(msg); });
  obstacles_subscribed_ = true;
}

void PgmAssistController::subscribeSpeed(const std::string & topic)
{
  if (!node_) {
    return;
  }
  unsubscribeSpeed();
  speed_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    topic, rclcpp::QoS(10).best_effort(),
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) { onSpeed(msg); });
  speed_subscribed_ = true;
}

void PgmAssistController::unsubscribePose()
{
  pose_sub_.reset();
  pose_subscribed_ = false;
}
void PgmAssistController::unsubscribeScan()
{
  scan_sub_.reset();
  clearScanItems();
  scan_subscribed_ = false;
}
void PgmAssistController::unsubscribeObstacles()
{
  obstacles_sub_.reset();
  clearObstacleItems();
  obstacles_subscribed_ = false;
}
void PgmAssistController::unsubscribeSpeed()
{
  speed_sub_.reset();
  speed_subscribed_ = false;
}

bool PgmAssistController::eventFilter(QObject * obj, QEvent * event)
{
  if (!view_ || obj != view_->viewport() || image_.isNull()) {
    return QObject::eventFilter(obj, event);
  }
  if (tool_mode_ == ToolMode::Idle) {
    return QObject::eventFilter(obj, event);
  }

  if (tool_mode_ == ToolMode::Brush) {
    if (event->type() == QEvent::MouseButtonPress) {
      auto * me = static_cast<QMouseEvent *>(event);
      if (me->button() == Qt::LeftButton) {
        pushUndo();
        painting_ = true;
        const QPointF sp = view_->mapToScene(me->pos());
        paintAt(sp);
        updateBrushOverlay(sp);
        return true;
      }
    } else if (event->type() == QEvent::MouseMove && painting_) {
      auto * me = static_cast<QMouseEvent *>(event);
      const QPointF sp = view_->mapToScene(me->pos());
      paintAt(sp);
      updateBrushOverlay(sp);
      return true;
    } else if (event->type() == QEvent::MouseButtonRelease) {
      auto * me = static_cast<QMouseEvent *>(event);
      if (me->button() == Qt::LeftButton) {
        painting_ = false;
        clearBrushOverlay();
        return true;
      }
    }
  }

  if (tool_mode_ == ToolMode::PathDraw) {
    if (event->type() == QEvent::MouseButtonPress) {
      auto * me = static_cast<QMouseEvent *>(event);
      if (me->button() == Qt::LeftButton) {
        addPathEndpoint(view_->mapToScene(me->pos()));
        return true;
      }
      if (me->button() == Qt::RightButton) {
        preview_enabled_ = false;
        updatePathPreview();
        return true;
      }
    } else if (event->type() == QEvent::MouseMove) {
      auto * me = static_cast<QMouseEvent *>(event);
      const QPointF sp = view_->mapToScene(me->pos());
      updatePathPreview(&sp);
      return false;  // allow pan with middle etc.
    }
  }
  return QObject::eventFilter(obj, event);
}

QWidget * PgmAssistController::createPanel(QWidget * parent)
{
  QWidget * page = new QWidget(parent);
  QVBoxLayout * lay = new QVBoxLayout(page);
  lay->setContentsMargins(6, 6, 6, 6);
  lay->setSpacing(6);

  QLabel * hint = new QLabel(
    QStringLiteral("仅在已加载 PGM 时可用。画笔将占用点擦为 free(254)。"));
  hint->setWordWrap(true);
  hint->setStyleSheet(QStringLiteral("color:#546e7a; font-size:11px;"));
  lay->addWidget(hint);

  QHBoxLayout * toolRow = new QHBoxLayout();
  brush_btn_ = new QToolButton();
  brush_btn_->setText(QStringLiteral("画笔擦除"));
  brush_btn_->setCheckable(true);
  path_btn_ = new QToolButton();
  path_btn_->setText(QStringLiteral("绘制路径"));
  path_btn_->setCheckable(true);
  QPushButton * idleBtn = new QPushButton(QStringLiteral("退出工具"));
  toolRow->addWidget(brush_btn_);
  toolRow->addWidget(path_btn_);
  toolRow->addWidget(idleBtn);
  lay->addLayout(toolRow);

  connect(brush_btn_, &QToolButton::toggled, this, [this](bool on) {
    if (on) {
      if (!hasPgmMap()) {
        brush_btn_->setChecked(false);
        emit statusMessage(QStringLiteral("请先加载 PGM 地图"));
        return;
      }
      setToolMode(ToolMode::Brush);
    } else if (tool_mode_ == ToolMode::Brush) {
      setToolMode(ToolMode::Idle);
    }
  });
  connect(path_btn_, &QToolButton::toggled, this, [this](bool on) {
    if (on) {
      if (!hasPgmMap()) {
        path_btn_->setChecked(false);
        emit statusMessage(QStringLiteral("请先加载 PGM 地图"));
        return;
      }
      setToolMode(ToolMode::PathDraw);
    } else if (tool_mode_ == ToolMode::PathDraw) {
      setToolMode(ToolMode::Idle);
    }
  });
  connect(idleBtn, &QPushButton::clicked, this, [this]() { setToolMode(ToolMode::Idle); });

  QHBoxLayout * brushRow = new QHBoxLayout();
  brushRow->addWidget(new QLabel(QStringLiteral("画笔尺寸")));
  brush_slider_ = new QSlider(Qt::Horizontal);
  brush_slider_->setRange(1, 100);
  brush_slider_->setValue(brush_radius_);
  connect(brush_slider_, &QSlider::valueChanged, this, [this](int v) { setBrushRadius(v); });
  brushRow->addWidget(brush_slider_);
  lay->addLayout(brushRow);

  QHBoxLayout * editRow = new QHBoxLayout();
  QPushButton * undoBtn = new QPushButton(QStringLiteral("撤销擦除"));
  QPushButton * savePgmBtn = new QPushButton(QStringLiteral("保存 PGM"));
  editRow->addWidget(undoBtn);
  editRow->addWidget(savePgmBtn);
  lay->addLayout(editRow);
  connect(undoBtn, &QPushButton::clicked, this, [this]() {
    if (!undoBrush()) {
      emit statusMessage(QStringLiteral("没有可撤销的擦除"));
    }
  });
  connect(savePgmBtn, &QPushButton::clicked, this, [this]() {
    if (!hasPgmMap()) {
      return;
    }
    const QString path = QFileDialog::getSaveFileName(
      nullptr, QStringLiteral("保存 PGM"), map_path_, QStringLiteral("PGM (*.pgm)"));
    if (path.isEmpty()) {
      return;
    }
    QString err;
    if (!savePgm(path, err)) {
      QMessageBox::warning(nullptr, QStringLiteral("保存 PGM"), err);
    }
  });

  QHBoxLayout * pathRow = new QHBoxLayout();
  QPushButton * savePathBtn = new QPushButton(QStringLiteral("路径保存"));
  QPushButton * openPathBtn = new QPushButton(QStringLiteral("打开路径"));
  QPushButton * clearPathBtn = new QPushButton(QStringLiteral("清除路径"));
  pathRow->addWidget(savePathBtn);
  pathRow->addWidget(openPathBtn);
  pathRow->addWidget(clearPathBtn);
  lay->addLayout(pathRow);
  connect(savePathBtn, &QPushButton::clicked, this, [this]() {
    if (!hasPgmMap()) {
      return;
    }
    QFileInfo fi(map_path_);
    const QString path = QFileDialog::getSaveFileName(
      nullptr, QStringLiteral("保存路径"), fi.absolutePath() + "/cdf_path", QStringLiteral("Path (*)"));
    if (path.isEmpty()) {
      return;
    }
    QString err;
    if (!savePath(path, err)) {
      QMessageBox::warning(nullptr, QStringLiteral("路径保存"), err);
    }
  });
  connect(openPathBtn, &QPushButton::clicked, this, [this]() {
    if (!hasPgmMap()) {
      return;
    }
    const QString path = QFileDialog::getOpenFileName(
      nullptr, QStringLiteral("打开路径"), QFileInfo(map_path_).absolutePath(), QStringLiteral("Path (*)"));
    if (path.isEmpty()) {
      return;
    }
    QString err;
    if (!openPath(path, err)) {
      QMessageBox::warning(nullptr, QStringLiteral("打开路径"), err);
    }
  });
  connect(clearPathBtn, &QPushButton::clicked, this, [this]() {
    clearPathItems(false);
    clearPersistentPath();
  });

  show_path_chk_ = new QCheckBox(QStringLiteral("显示路径"));
  show_path_chk_->setChecked(true);
  show_points_chk_ = new QCheckBox(QStringLiteral("显示点"));
  show_points_chk_->setChecked(true);
  show_arrows_chk_ = new QCheckBox(QStringLiteral("显示箭头"));
  show_arrows_chk_->setChecked(true);
  lay->addWidget(show_path_chk_);
  lay->addWidget(show_points_chk_);
  lay->addWidget(show_arrows_chk_);
  connect(show_path_chk_, &QCheckBox::toggled, this, &PgmAssistController::setShowPath);
  connect(show_points_chk_, &QCheckBox::toggled, this, &PgmAssistController::setShowPoints);
  connect(show_arrows_chk_, &QCheckBox::toggled, this, &PgmAssistController::setShowArrows);

  point_density_spin_ = new QSpinBox();
  point_density_spin_->setRange(1, 100);
  point_density_spin_->setValue(1);
  arrow_density_spin_ = new QSpinBox();
  arrow_density_spin_->setRange(1, 100);
  arrow_density_spin_->setValue(10);
  QFormLayout * densForm = new QFormLayout();
  densForm->addRow(QStringLiteral("点密度"), point_density_spin_);
  densForm->addRow(QStringLiteral("箭头密度"), arrow_density_spin_);
  lay->addLayout(densForm);
  connect(point_density_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &PgmAssistController::setPointDensity);
  connect(arrow_density_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &PgmAssistController::setArrowDensity);

  QGroupBox * rosBox = new QGroupBox(QStringLiteral("ROS2 叠加"));
  QVBoxLayout * rosLay = new QVBoxLayout(rosBox);
  topic_table_ = new QTableWidget(3, 3);
  topic_table_->setHorizontalHeaderLabels(
    {QStringLiteral("话题"), QStringLiteral("类型"), QStringLiteral("操作")});
  topic_table_->horizontalHeader()->setStretchLastSection(true);
  topic_table_->setItem(0, 0, new QTableWidgetItem(QStringLiteral("amcl_pose")));
  topic_table_->setItem(0, 1, new QTableWidgetItem(QStringLiteral("PoseWithCovarianceStamped")));
  topic_table_->setItem(1, 0, new QTableWidgetItem(QStringLiteral("scan")));
  topic_table_->setItem(1, 1, new QTableWidgetItem(QStringLiteral("LaserScan")));
  topic_table_->setItem(2, 0, new QTableWidgetItem(QStringLiteral("obstacles")));
  topic_table_->setItem(2, 1, new QTableWidgetItem(QStringLiteral("PointCloud2")));
  for (int row = 0; row < 3; ++row) {
    QPushButton * btn = new QPushButton(QStringLiteral("订阅"));
    topic_table_->setCellWidget(row, 2, btn);
    connect(btn, &QPushButton::clicked, this, [this, row, btn]() {
      if (!node_) {
        emit statusMessage(QStringLiteral("ROS node 未就绪"));
        return;
      }
      if (!hasPgmMap()) {
        emit statusMessage(QStringLiteral("请先加载 PGM"));
        return;
      }
      const QString topic = topic_table_->item(row, 0)->text();
      if (row == 0) {
        if (pose_subscribed_) {
          unsubscribePose();
          btn->setText(QStringLiteral("订阅"));
        } else {
          subscribePose(topic.toStdString());
          btn->setText(QStringLiteral("取消"));
        }
      } else if (row == 1) {
        if (scan_subscribed_) {
          unsubscribeScan();
          btn->setText(QStringLiteral("订阅"));
        } else {
          subscribeScan(topic.toStdString());
          btn->setText(QStringLiteral("取消"));
        }
      } else {
        if (obstacles_subscribed_) {
          unsubscribeObstacles();
          btn->setText(QStringLiteral("订阅"));
        } else {
          subscribeObstacles(topic.toStdString());
          btn->setText(QStringLiteral("取消"));
        }
      }
    });
  }
  topic_table_->setMaximumHeight(120);
  rosLay->addWidget(topic_table_);

  QHBoxLayout * speedRow = new QHBoxLayout();
  speed_topic_edit_ = new QLineEdit(QStringLiteral("cmd_vel"));
  speed_sub_btn_ = new QPushButton(QStringLiteral("订阅速度"));
  speedRow->addWidget(speed_topic_edit_);
  speedRow->addWidget(speed_sub_btn_);
  rosLay->addLayout(speedRow);
  connect(speed_sub_btn_, &QPushButton::clicked, this, [this]() {
    if (!node_) {
      return;
    }
    if (speed_subscribed_) {
      unsubscribeSpeed();
      speed_sub_btn_->setText(QStringLiteral("订阅速度"));
    } else {
      subscribeSpeed(speed_topic_edit_->text().trimmed().toStdString());
      speed_sub_btn_->setText(QStringLiteral("取消订阅"));
    }
  });

  speed_plot_ = new SpeedPlot();
  rosLay->addWidget(speed_plot_);
  lay->addWidget(rosBox);

  log_text_ = new QTextEdit();
  log_text_->setReadOnly(true);
  log_text_->setMaximumHeight(100);
  lay->addWidget(new QLabel(QStringLiteral("日志")));
  lay->addWidget(log_text_);
  connect(this, &PgmAssistController::statusMessage, this, [this](const QString & m) {
    if (log_text_) {
      log_text_->append(m);
    }
  });

  lay->addStretch(1);
  return page;
}
