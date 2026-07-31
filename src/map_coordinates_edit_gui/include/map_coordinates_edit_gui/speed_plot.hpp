#ifndef MAP_COORDINATES_EDIT_GUI_SPEED_PLOT_HPP_
#define MAP_COORDINATES_EDIT_GUI_SPEED_PLOT_HPP_

#include <QWidget>
#include <vector>

class SpeedPlot : public QWidget
{
  Q_OBJECT
public:
  explicit SpeedPlot(QWidget * parent = nullptr);

  void addData(double time, double linear_x, double linear_y, double angular_z);
  void clear();

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  std::vector<double> times_;
  std::vector<double> linear_x_;
  std::vector<double> linear_y_;
  std::vector<double> angular_z_;
};

#endif  // MAP_COORDINATES_EDIT_GUI_SPEED_PLOT_HPP_
