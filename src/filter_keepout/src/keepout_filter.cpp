#include "filter_keepout/keepout_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "sqlite3.h"

PLUGINLIB_EXPORT_CLASS(filter_keepout::KeepoutFilter, nav2_costmap_2d::Layer)

namespace filter_keepout
{

namespace
{
std::string expandUserPath(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  if (!home) {
    return path;
  }
  if (path.size() == 1) {
    return std::string(home);
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}
}  // namespace

KeepoutFilter::KeepoutFilter()
: nav2_costmap_2d::CostmapFilter()
{
}

void KeepoutFilter::initializeFilter(const std::string & filter_info_topic)
{
  (void)filter_info_topic;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("KeepoutFilter: failed to lock lifecycle node");
  }

  // Parameters under this filter's namespace (e.g. keepout_filter.use_sql)
  declareParameter("use_sql", rclcpp::ParameterValue(true));
  declareParameter("sql_db_path", rclcpp::ParameterValue(std::string("~/gps_filter_ws/data/keepout.db")));
  declareParameter("sql_table", rclcpp::ParameterValue(std::string("keepout_figures")));
  declareParameter("map_name", rclcpp::ParameterValue(std::string("")));
  declareParameter("fill_polygons", rclcpp::ParameterValue(true));
  declareParameter("line_thickness", rclcpp::ParameterValue(0.03));
  declareParameter("inflate", rclcpp::ParameterValue(true));
  declareParameter("inflation_radius", rclcpp::ParameterValue(1.0));
  declareParameter("inscribed_radius", rclcpp::ParameterValue(0.40));
  declareParameter("cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declareParameter("prohibition_topic", rclcpp::ParameterValue(std::string("prohibition_areas")));
  declareParameter("refresh_topic", rclcpp::ParameterValue(std::string("keepout_refresh")));

  node->get_parameter(getFullName("use_sql"), use_sql_);
  node->get_parameter(getFullName("sql_db_path"), sql_db_path_);
  node->get_parameter(getFullName("sql_table"), sql_table_);
  node->get_parameter(getFullName("map_name"), map_name_);
  node->get_parameter(getFullName("fill_polygons"), fill_polygons_);
  node->get_parameter(getFullName("line_thickness"), line_thickness_);
  node->get_parameter(getFullName("inflate"), inflate_);
  node->get_parameter(getFullName("inflation_radius"), inflation_radius_);
  node->get_parameter(getFullName("inscribed_radius"), inscribed_radius_);
  node->get_parameter(getFullName("cost_scaling_factor"), cost_scaling_factor_);
  node->get_parameter(getFullName("prohibition_topic"), prohibition_topic_);
  node->get_parameter(getFullName("refresh_topic"), refresh_topic_);

  sql_db_path_ = expandUserPath(sql_db_path_);

  RCLCPP_INFO(
    logger_,
    "KeepoutFilter init: use_sql=%d db=%s table=%s map_name='%s' fill=%d "
    "inflate=%d inflation_r=%.2f inscribed_r=%.2f",
    use_sql_ ? 1 : 0, sql_db_path_.c_str(), sql_table_.c_str(),
    map_name_.c_str(), fill_polygons_ ? 1 : 0,
    inflate_ ? 1 : 0, inflation_radius_, inscribed_radius_);

  if (use_sql_) {
    if (map_name_.empty()) {
      RCLCPP_WARN(
        logger_,
        "KeepoutFilter: map_name is empty — no SQL keepouts loaded. "
        "Set keepout_filter.map_name or publish keepout_refresh with 'map:<name>' when switching maps.");
    } else {
      // Nav2 clear_costmap calls reset()→initializeFilter often. Reuse memory if already loaded
      // so keepouts do not flicker off while SQLite reloads.
      const bool already_loaded =
        !lines_.empty() || !polygons_.empty() || !circles_.empty();
      if (already_loaded) {
        RCLCPP_INFO(
          logger_,
          "KeepoutFilter: reuse in-memory keepouts (lines=%zu polygons=%zu circles=%zu) after costmap reset",
          lines_.size(), polygons_.size(), circles_.size());
      } else if (!loadFromSqlite()) {
        RCLCPP_WARN(logger_, "KeepoutFilter: initial SQLite load failed or empty");
      }
    }
  }

  // Subscribe on the costmap lifecycle node (already spun by Nav2)
  rclcpp::SubscriptionOptions sub_opts;
  if (callback_group_) {
    sub_opts.callback_group = callback_group_;
  }

  prohibition_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    prohibition_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&KeepoutFilter::prohibitionCallback, this, std::placeholders::_1),
    sub_opts);

  refresh_sub_ = node->create_subscription<std_msgs::msg::String>(
    refresh_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&KeepoutFilter::refreshCallback, this, std::placeholders::_1),
    sub_opts);

  recomputeActive();
}

void KeepoutFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void)robot_x;
  (void)robot_y;
  (void)robot_yaw;
  std::lock_guard<std::mutex> lock(mutex_);

  auto expand = [&](double x, double y) {
      if (x < *min_x) {*min_x = x;}
      if (y < *min_y) {*min_y = y;}
      if (x > *max_x) {*max_x = x;}
      if (y > *max_y) {*max_y = y;}
    };

  for (const auto & p : prohibition_poses_) {
    expand(p.position.x, p.position.y);
  }
  for (const auto & line : lines_) {
    for (const auto & p : line) {
      expand(p.x, p.y);
    }
  }
  for (const auto & poly : polygons_) {
    for (const auto & p : poly) {
      expand(p.x, p.y);
    }
  }
  const double pad = (inflate_ && inflation_radius_ > 0.0) ? inflation_radius_ : 0.0;
  for (const auto & c : circles_) {
    const double r = c.radius + pad;
    expand(c.cx - r, c.cy - r);
    expand(c.cx + r, c.cy + r);
  }
  if (pad > 0.0) {
    for (const auto & p : prohibition_poses_) {
      expand(p.position.x - pad, p.position.y - pad);
      expand(p.position.x + pad, p.position.y + pad);
    }
    for (const auto & line : lines_) {
      for (const auto & p : line) {
        expand(p.x - pad, p.y - pad);
        expand(p.x + pad, p.y + pad);
      }
    }
    for (const auto & poly : polygons_) {
      for (const auto & p : poly) {
        expand(p.x - pad, p.y - pad);
        expand(p.x + pad, p.y + pad);
      }
    }
  }
}

void KeepoutFilter::process(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & pose)
{
  (void)pose;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return;
  }

  const double res = master_grid.getResolution();
  const double point_r = res * 1.5;

  // PoseArray overlay: small disks
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      for (const auto & p : prohibition_poses_) {
        const double dx = wx - p.position.x;
        const double dy = wy - p.position.y;
        if (std::sqrt(dx * dx + dy * dy) <= point_r) {
          master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
          break;
        }
      }
    }
  }

  for (const auto & line : lines_) {
    markPolyline(master_grid, line, false, false, min_i, min_j, max_i, max_j);
    if (line_thickness_ > 0.0 && line.size() >= 2) {
      for (size_t k = 0; k + 1 < line.size(); ++k) {
        const double x0 = line[k].x, y0 = line[k].y;
        const double x1 = line[k + 1].x, y1 = line[k + 1].y;
        const double len = std::hypot(x1 - x0, y1 - y0);
        const int steps = std::max(1, static_cast<int>(std::ceil(len / res)));
        for (int s = 0; s <= steps; ++s) {
          const double t = static_cast<double>(s) / steps;
          KeepoutCircle c;
          c.cx = x0 + t * (x1 - x0);
          c.cy = y0 + t * (y1 - y0);
          c.radius = line_thickness_;
          markCircle(master_grid, c, min_i, min_j, max_i, max_j);
        }
      }
    }
  }

  for (const auto & poly : polygons_) {
    if (fill_polygons_) {
      markPolygonFill(master_grid, poly, min_i, min_j, max_i, max_j);
    } else {
      markPolyline(master_grid, poly, true, false, min_i, min_j, max_i, max_j);
    }
  }

  for (const auto & c : circles_) {
    markCircle(master_grid, c, min_i, min_j, max_i, max_j);
  }

  if (inflate_ && inflation_radius_ > 0.0) {
    // Only inflate from keepout boundary cells (interior seeds are redundant & expensive)
    std::vector<PointInt> boundary_seeds;
    boundary_seeds.reserve(512);
    auto is_lethal = [&](int i, int j) {
        return master_grid.getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE;
      };
    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        if (!is_lethal(i, j)) {
          continue;
        }
        const bool edge =
          (i == min_i || !is_lethal(i - 1, j)) ||
          (i + 1 >= max_i || !is_lethal(i + 1, j)) ||
          (j == min_j || !is_lethal(i, j - 1)) ||
          (j + 1 >= max_j || !is_lethal(i, j + 1));
        if (edge) {
          boundary_seeds.push_back(PointInt{i, j});
        }
      }
    }
    if (!boundary_seeds.empty()) {
      inflateAroundSeeds(master_grid, boundary_seeds, min_i, min_j, max_i, max_j);
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<std::mutex> lock(mutex_);
  prohibition_poses_.clear();
  // Keep SQL geometry. clear_entirely_*_costmap → resetLayers() → reset() which
  // calls resetFilter(); wiping geometry here made keepouts flicker every recovery.
  prohibition_sub_.reset();
  refresh_sub_.reset();
  recomputeActive();
}

bool KeepoutFilter::isActive()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

void KeepoutFilter::prohibitionCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  prohibition_poses_ = msg->poses;
  recomputeActive();
}

void KeepoutFilter::refreshCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::string data = msg->data;
  // trim whitespace
  while (!data.empty() && (data.front() == ' ' || data.front() == '\t' ||
    data.front() == '\n' || data.front() == '\r'))
  {
    data.erase(data.begin());
  }
  while (!data.empty() && (data.back() == ' ' || data.back() == '\t' ||
    data.back() == '\n' || data.back() == '\r'))
  {
    data.pop_back();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!use_sql_) {
    RCLCPP_WARN(logger_, "KeepoutFilter: refresh requested but use_sql is false");
    return;
  }

  // Supported payloads:
  //   refresh | reload              — reload current map_name
  //   map:<name> | map_name=<name>  — switch active map and reload
  //   <name>                        — treat as map name if not a known verb
  bool switch_map = false;
  std::string new_map;
  if (data == "refresh" || data == "refreshCostMapPlayer" || data == "reload") {
    // keep map_name_
  } else if (data.rfind("map:", 0) == 0) {
    new_map = data.substr(4);
    switch_map = true;
  } else if (data.rfind("map_name=", 0) == 0) {
    new_map = data.substr(9);
    switch_map = true;
  } else if (!data.empty()) {
    new_map = data;
    switch_map = true;
  } else {
    RCLCPP_WARN(logger_, "KeepoutFilter: empty refresh command");
    return;
  }

  if (switch_map) {
    // trim new_map
    while (!new_map.empty() && (new_map.front() == ' ' || new_map.front() == '\t')) {
      new_map.erase(new_map.begin());
    }
    while (!new_map.empty() && (new_map.back() == ' ' || new_map.back() == '\t')) {
      new_map.pop_back();
    }
    if (new_map.empty()) {
      RCLCPP_WARN(logger_, "KeepoutFilter: map switch requested with empty name");
      return;
    }
    map_name_ = new_map;
    RCLCPP_INFO(logger_, "KeepoutFilter: active map_name set to '%s'", map_name_.c_str());
  }

  if (map_name_.empty()) {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: map_name is empty — set param keepout_filter.map_name "
      "or publish keepout_refresh with data 'map:<name>'");
    clearSqlGeometry();
    recomputeActive();
    return;
  }

  if (!loadFromSqlite()) {
    RCLCPP_WARN(logger_, "KeepoutFilter: refresh SQLite load failed for map '%s'", map_name_.c_str());
  }
  recomputeActive();
  RCLCPP_INFO(
    logger_, "KeepoutFilter refreshed map='%s': lines=%zu polygons=%zu circles=%zu",
    map_name_.c_str(), lines_.size(), polygons_.size(), circles_.size());
}

void KeepoutFilter::clearSqlGeometry()
{
  lines_.clear();
  polygons_.clear();
  circles_.clear();
}

void KeepoutFilter::recomputeActive()
{
  active_ = !prohibition_poses_.empty() || !lines_.empty() ||
    !polygons_.empty() || !circles_.empty();
}

bool KeepoutFilter::loadFromSqlite()
{
  if (map_name_.empty()) {
    RCLCPP_WARN(
      logger_,
      "KeepoutFilter: refusing to load all maps — set map_name to the current navigation map");
    return false;
  }

  sqlite3 * db = nullptr;
  if (sqlite3_open_v2(sql_db_path_.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(
      logger_, "KeepoutFilter: cannot open SQLite '%s': %s",
      sql_db_path_.c_str(), db ? sqlite3_errmsg(db) : "null");
    if (db) {
      sqlite3_close(db);
    }
    return false;
  }

  // Exact map_name match only — each navigation map has its own keepout set.
  const std::string sql =
    "SELECT figure_type, figure_name, center_x, center_y, radius, vertices_json "
    "FROM " + sql_table_ + " WHERE map_name = ? ORDER BY id;";

  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(logger_, "KeepoutFilter: prepare failed: %s", sqlite3_errmsg(db));
    sqlite3_close(db);
    return false;
  }

  sqlite3_bind_text(stmt, 1, map_name_.c_str(), -1, SQLITE_TRANSIENT);

  // Load into temps, then swap — never leave geometry empty mid-reload (avoids flicker)
  std::vector<std::vector<geometry_msgs::msg::Point>> new_lines;
  std::vector<std::vector<geometry_msgs::msg::Point>> new_polygons;
  std::vector<KeepoutCircle> new_circles;

  int rows = 0;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const char * type_c = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    const std::string type = type_c ? type_c : "";
    const char * verts_c = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));
    const std::string verts_json = verts_c ? verts_c : "[]";

    if (type == "circle") {
      KeepoutCircle c;
      c.cx = sqlite3_column_double(stmt, 2);
      c.cy = sqlite3_column_double(stmt, 3);
      c.radius = sqlite3_column_double(stmt, 4);
      if (c.radius > 0.0) {
        new_circles.push_back(c);
        ++rows;
      }
    } else if (type == "line") {
      auto pts = parseVerticesJson(verts_json);
      if (pts.size() >= 2) {
        new_lines.push_back(std::move(pts));
        ++rows;
      }
    } else if (type == "polygon" || type == "rectangle") {
      auto pts = parseVerticesJson(verts_json);
      if (pts.size() >= 3) {
        new_polygons.push_back(std::move(pts));
        ++rows;
      }
    } else {
      RCLCPP_WARN(logger_, "KeepoutFilter: unknown figure_type '%s'", type.c_str());
    }
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);

  lines_.swap(new_lines);
  polygons_.swap(new_polygons);
  circles_.swap(new_circles);

  RCLCPP_INFO(
    logger_, "KeepoutFilter loaded %d figure(s) for map_name='%s'",
    rows, map_name_.c_str());
  return true;  // empty map (0 figures) is valid
}

std::vector<geometry_msgs::msg::Point> KeepoutFilter::parseVerticesJson(const std::string & json)
{
  // Expect [[x,y],[x,y],...]
  std::vector<geometry_msgs::msg::Point> out;
  size_t i = 0;
  const size_t n = json.size();
  auto skipWs = [&]() {
      while (i < n && (json[i] == ' ' || json[i] == '\n' || json[i] == '\t' || json[i] == '\r')) {
        ++i;
      }
    };

  skipWs();
  if (i >= n || json[i] != '[') {
    return out;
  }
  ++i;

  while (i < n) {
    skipWs();
    if (i < n && json[i] == ']') {
      break;
    }
    if (i < n && json[i] == ',') {
      ++i;
      continue;
    }
    if (i >= n || json[i] != '[') {
      break;
    }
    ++i;
    skipWs();
    size_t start = i;
    while (i < n && json[i] != ',' && json[i] != ']') {
      ++i;
    }
    double x = 0.0, y = 0.0;
    try {
      x = std::stod(json.substr(start, i - start));
    } catch (...) {
      break;
    }
    if (i < n && json[i] == ',') {
      ++i;
    }
    skipWs();
    start = i;
    while (i < n && json[i] != ']') {
      ++i;
    }
    try {
      y = std::stod(json.substr(start, i - start));
    } catch (...) {
      break;
    }
    if (i < n && json[i] == ']') {
      ++i;
    }
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    out.push_back(p);
  }
  return out;
}

void KeepoutFilter::markCircle(
  nav2_costmap_2d::Costmap2D & master_grid,
  const KeepoutCircle & circle,
  int min_i, int min_j, int max_i, int max_j,
  std::vector<PointInt> * /*lethal_seeds*/) const
{
  const double r2 = circle.radius * circle.radius;
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      const double dx = wx - circle.cx;
      const double dy = wy - circle.cy;
      if (dx * dx + dy * dy <= r2) {
        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void KeepoutFilter::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> & cells) const
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  PointInt pt;
  pt.x = x0;
  pt.y = y0;
  int n = 1 + dx + dy;
  int x_inc = (x1 > x0) ? 1 : -1;
  int y_inc = (y1 > y0) ? 1 : -1;
  int error = dx - dy;
  dx *= 2;
  dy *= 2;

  for (; n > 0; --n) {
    cells.push_back(pt);
    if (error > 0) {
      pt.x += x_inc;
      error -= dy;
    } else {
      pt.y += y_inc;
      error += dx;
    }
  }
}

void KeepoutFilter::polygonOutlineCells(
  const std::vector<PointInt> & polygon, std::vector<PointInt> & cells) const
{
  if (polygon.empty()) {
    return;
  }
  for (size_t i = 0; i + 1 < polygon.size(); ++i) {
    raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, cells);
  }
  raytrace(
    polygon.back().x, polygon.back().y,
    polygon.front().x, polygon.front().y, cells);
}

void KeepoutFilter::rasterizePolygon(
  const std::vector<PointInt> & polygon, std::vector<PointInt> & cells, bool fill) const
{
  if (polygon.size() < 3) {
    return;
  }
  polygonOutlineCells(polygon, cells);
  if (!fill || cells.size() < 2) {
    return;
  }

  // Bubble-sort by x (same approach as ROS1 costmap_prohibition_layer)
  for (size_t pass = 0; pass + 1 < cells.size(); ++pass) {
    for (size_t i = 0; i + 1 < cells.size() - pass; ++i) {
      if (cells[i].x > cells[i + 1].x) {
        std::swap(cells[i], cells[i + 1]);
      }
    }
  }

  size_t i = 0;
  const int min_x = cells.front().x;
  const int max_x = cells.back().x;
  for (int x = min_x; x <= max_x; ++x) {
    if (i >= cells.size() - 1) {
      break;
    }
    while (i < cells.size() && cells[i].x != x) {
      ++i;
    }
    if (i >= cells.size()) {
      break;
    }
    PointInt min_pt = cells[i];
    PointInt max_pt = min_pt;
    while (i < cells.size() && cells[i].x == x) {
      if (cells[i].y < min_pt.y) {min_pt = cells[i];}
      if (cells[i].y > max_pt.y) {max_pt = cells[i];}
      ++i;
    }
    for (int y = min_pt.y; y <= max_pt.y; ++y) {
      PointInt pt;
      pt.x = x;
      pt.y = y;
      cells.push_back(pt);
    }
  }
}

bool KeepoutFilter::pointInPolygon(
  double x, double y,
  const std::vector<geometry_msgs::msg::Point> & poly)
{
  // Ray casting; poly assumed closed (last edge back to first)
  const size_t n = poly.size();
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double yi = poly[i].y;
    const double yj = poly[j].y;
    const double xi = poly[i].x;
    const double xj = poly[j].x;
    const bool intersect =
      ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-18) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

void KeepoutFilter::markPolygonFill(
  nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<geometry_msgs::msg::Point> & points,
  int min_i, int min_j, int max_i, int max_j,
  std::vector<PointInt> * lethal_seeds) const
{
  if (points.size() < 3) {
    return;
  }

  // Cheap reject: skip window if it cannot overlap polygon AABB
  double pmin_x = points[0].x, pmax_x = points[0].x;
  double pmin_y = points[0].y, pmax_y = points[0].y;
  for (size_t k = 1; k < points.size(); ++k) {
    pmin_x = std::min(pmin_x, points[k].x);
    pmax_x = std::max(pmax_x, points[k].x);
    pmin_y = std::min(pmin_y, points[k].y);
    pmax_y = std::max(pmax_y, points[k].y);
  }
  double wmin_x, wmin_y, wmax_x, wmax_y;
  master_grid.mapToWorld(min_i, min_j, wmin_x, wmin_y);
  master_grid.mapToWorld(max_i - 1, max_j - 1, wmax_x, wmax_y);
  if (wmax_x < wmin_x) {std::swap(wmin_x, wmax_x);}
  if (wmax_y < wmin_y) {std::swap(wmin_y, wmax_y);}
  if (pmax_x < wmin_x || pmin_x > wmax_x || pmax_y < wmin_y || pmin_y > wmax_y) {
    return;
  }

  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      if (pointInPolygon(wx, wy, points)) {
        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void KeepoutFilter::markPolyline(
  nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<geometry_msgs::msg::Point> & points,
  bool closed,
  bool fill,
  int min_i, int min_j, int max_i, int max_j,
  std::vector<PointInt> * lethal_seeds) const
{
  if (points.size() < 2) {
    return;
  }

  // Filled closed shapes: use world-frame PIP (safe with rolling costmaps)
  if (closed && fill && points.size() >= 3) {
    markPolygonFill(master_grid, points, min_i, min_j, max_i, max_j, lethal_seeds);
    return;
  }

  std::vector<PointInt> map_poly;
  map_poly.reserve(points.size());
  for (const auto & p : points) {
    PointInt loc;
    master_grid.worldToMapNoBounds(p.x, p.y, loc.x, loc.y);
    map_poly.push_back(loc);
  }

  std::vector<PointInt> cells;
  for (size_t i = 0; i + 1 < map_poly.size(); ++i) {
    raytrace(map_poly[i].x, map_poly[i].y, map_poly[i + 1].x, map_poly[i + 1].y, cells);
  }
  if (closed && map_poly.size() >= 2) {
    raytrace(
      map_poly.back().x, map_poly.back().y,
      map_poly.front().x, map_poly.front().y, cells);
  }

  for (const auto & c : cells) {
    if (c.x < min_i || c.x >= max_i || c.y < min_j || c.y >= max_j) {
      continue;
    }
    master_grid.setCost(c.x, c.y, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

unsigned char KeepoutFilter::inflationCost(double distance) const
{
  if (distance < 0.0) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  if (distance <= inscribed_radius_) {
    return nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }
  if (distance > inflation_radius_) {
    return 0;
  }
  // Same formula as nav2_costmap_2d::InflationLayer
  const double factor = std::exp(-cost_scaling_factor_ * (distance - inscribed_radius_));
  return static_cast<unsigned char>(
    (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * (1.0 - factor));
}

void KeepoutFilter::inflateAroundSeeds(
  nav2_costmap_2d::Costmap2D & master_grid,
  const std::vector<PointInt> & seeds,
  int min_i, int min_j, int max_i, int max_j) const
{
  const double res = master_grid.getResolution();
  if (res <= 0.0 || inflation_radius_ <= 0.0) {
    return;
  }
  const int cell_r = static_cast<int>(std::ceil(inflation_radius_ / res));
  const double r2 = inflation_radius_ * inflation_radius_;

  for (const auto & seed : seeds) {
    for (int di = -cell_r; di <= cell_r; ++di) {
      for (int dj = -cell_r; dj <= cell_r; ++dj) {
        const int i = seed.x + di;
        const int j = seed.y + dj;
        if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
          continue;
        }
        const double dist = std::hypot(di * res, dj * res);
        if (dist * dist > r2) {
          continue;
        }
        const unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }
        const unsigned char new_cost = inflationCost(dist);
        if (new_cost > old_cost) {
          master_grid.setCost(i, j, new_cost);
        }
      }
    }
  }
}

}  // namespace filter_keepout
