#ifndef FILTER_KEEPOUT__KEEPOUT_FILTER_HPP_
#define FILTER_KEEPOUT__KEEPOUT_FILTER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace filter_keepout
{

struct KeepoutCircle
{
  double cx{0.0};
  double cy{0.0};
  double radius{0.0};
};

struct PointInt
{
  int x{0};
  int y{0};
};

/**
 * @brief Nav2 CostmapFilter that marks keepout regions from SQLite and/or PoseArray.
 *
 * SQLite schema: keepout_figures (line / polygon / circle in map meters).
 * Runs after inflation_layer, so it optionally self-inflates keepout lethals.
 */
class KeepoutFilter : public nav2_costmap_2d::CostmapFilter
{
public:
  KeepoutFilter();
  ~KeepoutFilter() override = default;

  void initializeFilter(const std::string & filter_info_topic) override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose) override;

  void resetFilter() override;

  bool isActive();

private:
  void prohibitionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void refreshCallback(const std_msgs::msg::String::SharedPtr msg);

  bool loadFromSqlite();
  void clearSqlGeometry();
  void recomputeActive();

  void markCircle(
    nav2_costmap_2d::Costmap2D & master_grid,
    const KeepoutCircle & circle,
    int min_i, int min_j, int max_i, int max_j,
    std::vector<PointInt> * lethal_seeds = nullptr) const;

  void markPolyline(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<geometry_msgs::msg::Point> & points,
    bool closed,
    bool fill,
    int min_i, int min_j, int max_i, int max_j,
    std::vector<PointInt> * lethal_seeds = nullptr) const;

  void markPolygonFill(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<geometry_msgs::msg::Point> & points,
    int min_i, int min_j, int max_i, int max_j,
    std::vector<PointInt> * lethal_seeds = nullptr) const;

  void inflateAroundSeeds(
    nav2_costmap_2d::Costmap2D & master_grid,
    const std::vector<PointInt> & seeds,
    int min_i, int min_j, int max_i, int max_j) const;

  unsigned char inflationCost(double distance) const;

  void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> & cells) const;
  void polygonOutlineCells(
    const std::vector<PointInt> & polygon, std::vector<PointInt> & cells) const;
  void rasterizePolygon(
    const std::vector<PointInt> & polygon, std::vector<PointInt> & cells, bool fill) const;

  static bool pointInPolygon(
    double x, double y,
    const std::vector<geometry_msgs::msg::Point> & poly);

  static std::vector<geometry_msgs::msg::Point> parseVerticesJson(const std::string & json);

  std::mutex mutex_;

  std::vector<geometry_msgs::msg::Pose> prohibition_poses_;
  std::vector<std::vector<geometry_msgs::msg::Point>> lines_;
  std::vector<std::vector<geometry_msgs::msg::Point>> polygons_;
  std::vector<KeepoutCircle> circles_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr prohibition_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr refresh_sub_;

  bool active_{false};
  bool use_sql_{true};
  bool fill_polygons_{true};
  double line_thickness_{0.03};  // meters; thickened via circle along line cells
  bool inflate_{true};
  double inflation_radius_{1.0};
  double inscribed_radius_{0.40};
  double cost_scaling_factor_{3.0};
  std::string sql_db_path_;
  std::string sql_table_{"keepout_figures"};
  std::string map_name_;
  std::string prohibition_topic_{"prohibition_areas"};
  std::string refresh_topic_{"keepout_refresh"};
};

}  // namespace filter_keepout

#endif  // FILTER_KEEPOUT__KEEPOUT_FILTER_HPP_
