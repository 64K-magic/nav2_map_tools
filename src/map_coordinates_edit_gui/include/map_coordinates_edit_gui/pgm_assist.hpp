#ifndef MAP_COORDINATES_EDIT_GUI_PGM_ASSIST_HPP_
#define MAP_COORDINATES_EDIT_GUI_PGM_ASSIST_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QObject>
#include <QPointF>
#include <QString>
#include <QWidget>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class SpeedPlot;
class QCheckBox;
class QComboBox;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTableWidget;
class QTextEdit;
class QToolButton;

/**
 * PGM assist tools ported from map_editor:
 * brush erase, path draw/save/load, ROS overlays (pose/scan/obstacles/cmd_vel).
 * Operates on the shared map scene/pixmap of map_coordinates_edit_gui.
 */
class PgmAssistController : public QObject
{
  Q_OBJECT
public:
  enum class ToolMode {
    Idle,
    Brush,
    PathDraw,
  };

  explicit PgmAssistController(QObject * parent = nullptr);
  ~PgmAssistController() override;

  /** Bind to the main map scene/view/pixmap. Call after UI is built. */
  void attach(QGraphicsScene * scene, QGraphicsPixmapItem * map_item, QGraphicsView * view);

  /** Use the shared ROS node for overlay subscriptions. */
  void setNode(const rclcpp::Node::SharedPtr & node);

  /**
   * Activate after a PGM is loaded. editable image is grayscale working copy.
   * map_path used for sibling YAML lookup when saving paths.
   */
  void setPgmMap(
    const QImage & editable, const QString & map_path,
    double resolution, double origin_x, double origin_y);

  /** Leave PGM mode (tile map): disable tools and clear overlays. */
  void clearPgmMap();

  bool hasPgmMap() const { return !image_.isNull(); }

  /** Build the sidebar page contents (caller owns returned widget). */
  QWidget * createPanel(QWidget * parent = nullptr);

  void setToolMode(ToolMode mode);
  ToolMode toolMode() const { return tool_mode_; }

  void setBrushRadius(int r);
  int brushRadius() const { return brush_radius_; }

  bool savePgm(const QString & path, QString & error);
  bool undoBrush();

  bool savePath(const QString & path, QString & error);
  bool openPath(const QString & path, QString & error);

  void setShowPath(bool on);
  void setShowPoints(bool on);
  void setShowArrows(bool on);
  void setPointDensity(int n);
  void setArrowDensity(int n);

signals:
  void statusMessage(const QString & msg);
  /** Emitted when brush/path tool claims exclusive mouse input. */
  void toolModeChanged(bool exclusive);
  void imageEdited();

protected:
  bool eventFilter(QObject * obj, QEvent * event) override;

private:
  void syncPixmap();
  void pushUndo();
  void paintAt(const QPointF & scene_pt);
  void updateBrushOverlay(const QPointF & scene_pt);
  void clearBrushOverlay();

  void clearPathItems(bool keep_persistent);
  void clearPersistentPath();
  void addPathEndpoint(const QPointF & p);
  void updatePathPreview(const QPointF * cursor = nullptr);
  int hitTestControlPoint(const QPointF & scene_pt) const;
  void updateControlPoint(int index, const QPointF & pos);
  std::vector<QPointF> samplePath(int samples_per_segment = 100) const;
  void drawPersistentPath(const std::vector<QPointF> & pts);
  void redrawPersistentPath();

  QPointF worldToScene(double x, double y) const;
  void sceneToWorld(const QPointF & scene_pt, double & x, double & y) const;
  bool loadYamlBesideMap(double & resolution, double & origin_x, double & origin_y) const;

  void updateRobotPose(double x, double y, double yaw);
  void updateScanPoints(const std::vector<QPointF> & world_pts);
  void updateObstaclePoints(const std::vector<QPointF> & world_pts);
  void clearScanItems();
  void clearObstacleItems();
  void clearRobotItems();

  void subscribePose(const std::string & topic);
  void subscribeScan(const std::string & topic);
  void subscribeObstacles(const std::string & topic);
  void subscribeSpeed(const std::string & topic);
  void unsubscribePose();
  void unsubscribeScan();
  void unsubscribeObstacles();
  void unsubscribeSpeed();

  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onSpeed(const geometry_msgs::msg::Twist::SharedPtr msg);

  static std::vector<std::tuple<double, double, double>> resamplePath(
    const std::vector<std::tuple<double, double, double>> & path, double target_distance);

  QGraphicsScene * scene_{nullptr};
  QGraphicsPixmapItem * map_item_{nullptr};
  QGraphicsView * view_{nullptr};
  rclcpp::Node::SharedPtr node_;

  QImage image_;
  QString map_path_;
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};

  ToolMode tool_mode_{ToolMode::Idle};
  int brush_radius_{8};
  int draw_value_{254};
  bool painting_{false};
  std::vector<QImage> undo_stack_;
  int max_undo_{10};
  QGraphicsEllipseItem * brush_overlay_{nullptr};

  // Path drawing
  std::vector<QPointF> end_points_;
  std::vector<QPointF> control_points_;
  std::vector<QGraphicsEllipseItem *> end_items_;
  std::vector<QGraphicsEllipseItem *> control_items_;
  QGraphicsPathItem * preview_path_{nullptr};
  bool preview_enabled_{true};
  bool dragging_control_{false};
  int dragging_index_{-1};
  std::vector<QPointF> persistent_pts_;
  QGraphicsPathItem * persistent_path_{nullptr};
  std::vector<QGraphicsEllipseItem *> path_point_items_;
  std::vector<QGraphicsPolygonItem *> path_arrow_items_;
  bool show_path_{true};
  bool show_points_{true};
  bool show_arrows_{true};
  int point_density_{1};
  int arrow_density_{10};

  // ROS overlays
  QGraphicsEllipseItem * robot_item_{nullptr};
  QGraphicsPolygonItem * robot_arrow_{nullptr};
  std::vector<QGraphicsEllipseItem *> scan_items_;
  std::vector<QGraphicsEllipseItem *> obstacle_items_;
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_yaw_{0.0};
  bool have_robot_pose_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_sub_;

  // Panel widgets (not owned if parented to panel)
  QSlider * brush_slider_{nullptr};
  QToolButton * brush_btn_{nullptr};
  QToolButton * path_btn_{nullptr};
  QCheckBox * show_path_chk_{nullptr};
  QCheckBox * show_points_chk_{nullptr};
  QCheckBox * show_arrows_chk_{nullptr};
  QSpinBox * point_density_spin_{nullptr};
  QSpinBox * arrow_density_spin_{nullptr};
  QLineEdit * speed_topic_edit_{nullptr};
  QPushButton * speed_sub_btn_{nullptr};
  SpeedPlot * speed_plot_{nullptr};
  QTextEdit * log_text_{nullptr};
  QTableWidget * topic_table_{nullptr};
  bool pose_subscribed_{false};
  bool scan_subscribed_{false};
  bool obstacles_subscribed_{false};
  bool speed_subscribed_{false};
};

#endif  // MAP_COORDINATES_EDIT_GUI_PGM_ASSIST_HPP_
