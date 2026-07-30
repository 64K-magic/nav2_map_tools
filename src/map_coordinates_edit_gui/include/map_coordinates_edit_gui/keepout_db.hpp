#ifndef MAP_COORDINATES_EDIT_GUI_KEEPOUT_DB_HPP_
#define MAP_COORDINATES_EDIT_GUI_KEEPOUT_DB_HPP_

#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>

namespace keepout_db
{

struct Figure
{
  int id{-1};
  std::string figure_type;  // line | polygon | circle
  std::string figure_name;
  std::string map_name;
  double center_x{0.0};
  double center_y{0.0};
  double radius{0.0};
  std::vector<geometry_msgs::msg::Point> vertices;  // map meters
};

/** Ensure DB file exists and keepout_figures table is created. */
bool ensureSchema(const std::string & db_path, std::string & error);

/**
 * Replace all figures belonging to map_name, then insert.
 * map_name must be non-empty (one navigation map → many keepout rows).
 */
bool replaceFigures(
  const std::string & db_path,
  const std::string & map_name,
  const std::vector<Figure> & figures,
  std::string & error);

/**
 * Load figures for an exact map_name.
 * map_name must be non-empty.
 */
bool loadFigures(
  const std::string & db_path,
  const std::string & map_name,
  std::vector<Figure> & figures,
  std::string & error);

/** Distinct map_name values present in the DB (non-empty). */
bool listMapNames(
  const std::string & db_path,
  std::vector<std::string> & names,
  std::string & error);

std::string verticesToJson(const std::vector<geometry_msgs::msg::Point> & pts);
bool parseVerticesJson(const std::string & json, std::vector<geometry_msgs::msg::Point> & out);

}  // namespace keepout_db

#endif  // MAP_COORDINATES_EDIT_GUI_KEEPOUT_DB_HPP_
