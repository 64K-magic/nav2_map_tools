#include "map_coordinates_edit_gui/speed_plot.hpp"

#include <QPainter>
#include <algorithm>
#include <cmath>

SpeedPlot::SpeedPlot(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(160);
  setMinimumWidth(240);
}

void SpeedPlot::addData(double time, double linear_x, double linear_y, double angular_z)
{
  times_.push_back(time);
  linear_x_.push_back(linear_x);
  linear_y_.push_back(linear_y);
  angular_z_.push_back(angular_z);
  if (times_.size() > 1000) {
    times_.erase(times_.begin());
    linear_x_.erase(linear_x_.begin());
    linear_y_.erase(linear_y_.begin());
    angular_z_.erase(angular_z_.begin());
  }
  update();
}

void SpeedPlot::clear()
{
  times_.clear();
  linear_x_.clear();
  linear_y_.clear();
  angular_z_.clear();
  update();
}

void SpeedPlot::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.fillRect(rect(), Qt::white);
  painter.setPen(QPen(Qt::black, 1));
  painter.drawRect(rect().adjusted(0, 0, -1, -1));
  if (times_.empty()) {
    return;
  }

  const double minTime = *std::min_element(times_.begin(), times_.end());
  const double maxTime = *std::max_element(times_.begin(), times_.end());
  const double minSpeed = std::min({*std::min_element(linear_x_.begin(), linear_x_.end()),
                                    *std::min_element(linear_y_.begin(), linear_y_.end()),
                                    *std::min_element(angular_z_.begin(), angular_z_.end())});
  const double maxSpeed = std::max({*std::max_element(linear_x_.begin(), linear_x_.end()),
                                    *std::max_element(linear_y_.begin(), linear_y_.end()),
                                    *std::max_element(angular_z_.begin(), angular_z_.end())});
  double rangeTime = maxTime - minTime;
  double rangeSpeed = maxSpeed - minSpeed;
  if (rangeTime == 0.0) {
    rangeTime = 1.0;
  }
  if (rangeSpeed == 0.0) {
    rangeSpeed = 1.0;
  }

  painter.setPen(QPen(Qt::lightGray, 1, Qt::DotLine));
  for (int i = 0; i <= 10; ++i) {
    const int x = i * width() / 10;
    const int y = i * height() / 10;
    painter.drawLine(x, 0, x, height());
    painter.drawLine(0, y, width(), y);
  }

  painter.setPen(QPen(Qt::black, 2));
  painter.drawLine(0, height() / 2, width(), height() / 2);
  painter.drawLine(width() / 2, 0, width() / 2, height());

  auto drawSeries = [&](const std::vector<double> & series, const QColor & color) {
    painter.setPen(QPen(color, 2));
    for (size_t i = 1; i < times_.size(); ++i) {
      const int x1 = static_cast<int>(((times_[i - 1] - minTime) / rangeTime) * width());
      const int y1 = height() - static_cast<int>(((series[i - 1] - minSpeed) / rangeSpeed) * height());
      const int x2 = static_cast<int>(((times_[i] - minTime) / rangeTime) * width());
      const int y2 = height() - static_cast<int>(((series[i] - minSpeed) / rangeSpeed) * height());
      painter.drawLine(x1, y1, x2, y2);
    }
  };
  drawSeries(linear_x_, Qt::blue);
  drawSeries(linear_y_, Qt::green);
  drawSeries(angular_z_, Qt::red);

  painter.setPen(QPen(Qt::black, 1));
  painter.drawText(10, 20, QStringLiteral("Linear X (blue)"));
  painter.drawText(10, 40, QStringLiteral("Linear Y (green)"));
  painter.drawText(10, 60, QStringLiteral("Angular Z (red)"));
}
