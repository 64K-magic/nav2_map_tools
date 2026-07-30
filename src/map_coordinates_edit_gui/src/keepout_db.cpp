#include "map_coordinates_edit_gui/keepout_db.hpp"

#include <cstdlib>
#include <cstdio>
#include <sstream>

#include <sqlite3.h>

namespace keepout_db
{

namespace
{
std::string expandUser(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = getenv("HOME");
  if (!home) {
    return path;
  }
  if (path.size() == 1) {
    return home;
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}
}  // namespace

std::string verticesToJson(const std::vector<geometry_msgs::msg::Point> & pts)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i) {
      oss << ",";
    }
    oss << "[" << pts[i].x << "," << pts[i].y << "]";
  }
  oss << "]";
  return oss.str();
}

bool parseVerticesJson(const std::string & json, std::vector<geometry_msgs::msg::Point> & out)
{
  out.clear();
  size_t i = 0;
  const size_t n = json.size();
  auto skipWs = [&]() {
      while (i < n && (json[i] == ' ' || json[i] == '\n' || json[i] == '\t' || json[i] == '\r')) {
        ++i;
      }
    };
  skipWs();
  if (i >= n || json[i] != '[') {
    return false;
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
      return false;
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
      return false;
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
  return true;
}

bool ensureSchema(const std::string & db_path, std::string & error)
{
  const std::string path = expandUser(db_path);
  sqlite3 * db = nullptr;
  if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) {
    error = db ? sqlite3_errmsg(db) : "open failed";
    if (db) {
      sqlite3_close(db);
    }
    return false;
  }
  const char * sql =
    "CREATE TABLE IF NOT EXISTS keepout_figures ("
    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  figure_type TEXT NOT NULL,"
    "  figure_name TEXT,"
    "  map_name TEXT NOT NULL DEFAULT '',"
    "  center_x REAL,"
    "  center_y REAL,"
    "  radius REAL,"
    "  vertices_json TEXT NOT NULL DEFAULT '[]',"
    "  updated_at TEXT"
    ");"
    "CREATE INDEX IF NOT EXISTS idx_keepout_figures_map_name "
    "  ON keepout_figures(map_name);";
  char * err = nullptr;
  if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
    error = err ? err : "create table failed";
    sqlite3_free(err);
    sqlite3_close(db);
    return false;
  }
  sqlite3_close(db);
  return true;
}

bool replaceFigures(
  const std::string & db_path,
  const std::string & map_name,
  const std::vector<Figure> & figures,
  std::string & error)
{
  if (map_name.empty()) {
    error = "map_name is required (identify which navigation map these keepouts belong to)";
    return false;
  }
  if (!ensureSchema(db_path, error)) {
    return false;
  }
  const std::string path = expandUser(db_path);
  sqlite3 * db = nullptr;
  if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) {
    error = db ? sqlite3_errmsg(db) : "open failed";
    if (db) {
      sqlite3_close(db);
    }
    return false;
  }

  sqlite3_exec(db, "BEGIN;", nullptr, nullptr, nullptr);

  // Only replace keepouts for this map — leave other maps untouched.
  sqlite3_stmt * del = nullptr;
  if (sqlite3_prepare_v2(
      db, "DELETE FROM keepout_figures WHERE map_name = ?;", -1, &del, nullptr) != SQLITE_OK)
  {
    error = sqlite3_errmsg(db);
    sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
    sqlite3_close(db);
    return false;
  }
  sqlite3_bind_text(del, 1, map_name.c_str(), -1, SQLITE_TRANSIENT);
  if (sqlite3_step(del) != SQLITE_DONE) {
    error = sqlite3_errmsg(db);
    sqlite3_finalize(del);
    sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
    sqlite3_close(db);
    return false;
  }
  sqlite3_finalize(del);

  const char * ins =
    "INSERT INTO keepout_figures "
    "(figure_type, figure_name, map_name, center_x, center_y, radius, vertices_json, updated_at) "
    "VALUES (?,?,?,?,?,?,?,datetime('now'));";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, ins, -1, &stmt, nullptr) != SQLITE_OK) {
    error = sqlite3_errmsg(db);
    sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
    sqlite3_close(db);
    return false;
  }

  for (const auto & f : figures) {
    sqlite3_reset(stmt);
    sqlite3_clear_bindings(stmt);
    sqlite3_bind_text(stmt, 1, f.figure_type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, f.figure_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, map_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_double(stmt, 4, f.center_x);
    sqlite3_bind_double(stmt, 5, f.center_y);
    sqlite3_bind_double(stmt, 6, f.radius);
    const std::string json = verticesToJson(f.vertices);
    sqlite3_bind_text(stmt, 7, json.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      error = sqlite3_errmsg(db);
      sqlite3_finalize(stmt);
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      sqlite3_close(db);
      return false;
    }
  }

  sqlite3_finalize(stmt);
  sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
  sqlite3_close(db);
  return true;
}

bool loadFigures(
  const std::string & db_path,
  const std::string & map_name,
  std::vector<Figure> & figures,
  std::string & error)
{
  figures.clear();
  if (map_name.empty()) {
    error = "map_name is required to load keepouts for the current navigation map";
    return false;
  }

  const std::string path = expandUser(db_path);
  sqlite3 * db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    error = db ? sqlite3_errmsg(db) : "open failed";
    if (db) {
      sqlite3_close(db);
    }
    return false;
  }

  const char * sql =
    "SELECT id, figure_type, figure_name, map_name, center_x, center_y, radius, vertices_json "
    "FROM keepout_figures WHERE map_name = ? ORDER BY id;";

  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    error = sqlite3_errmsg(db);
    sqlite3_close(db);
    return false;
  }
  sqlite3_bind_text(stmt, 1, map_name.c_str(), -1, SQLITE_TRANSIENT);

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    Figure f;
    f.id = sqlite3_column_int(stmt, 0);
    const char * t = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    const char * n = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
    const char * m = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
    f.figure_type = t ? t : "";
    f.figure_name = n ? n : "";
    f.map_name = m ? m : "";
    f.center_x = sqlite3_column_double(stmt, 4);
    f.center_y = sqlite3_column_double(stmt, 5);
    f.radius = sqlite3_column_double(stmt, 6);
    const char * v = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 7));
    parseVerticesJson(v ? v : "[]", f.vertices);
    figures.push_back(std::move(f));
  }

  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return true;
}

bool listMapNames(
  const std::string & db_path,
  std::vector<std::string> & names,
  std::string & error)
{
  names.clear();
  const std::string path = expandUser(db_path);
  sqlite3 * db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    error = db ? sqlite3_errmsg(db) : "open failed";
    if (db) {
      sqlite3_close(db);
    }
    return false;
  }

  const char * sql =
    "SELECT DISTINCT map_name FROM keepout_figures "
    "WHERE map_name IS NOT NULL AND map_name != '' "
    "ORDER BY map_name;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    error = sqlite3_errmsg(db);
    sqlite3_close(db);
    return false;
  }
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const char * m = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    if (m && m[0] != '\0') {
      names.emplace_back(m);
    }
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  return true;
}

}  // namespace keepout_db
