#ifndef FILTER_KEEPOUT__KEEPOUT_EDITOR_HPP_
#define FILTER_KEEPOUT__KEEPOUT_EDITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QFileDialog>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QImage>
#include <QPointF>
#include <QObject>
#include <sstream>
#include <cstdio>

namespace filter_keepout {

// simple Qt widget that allows entering a comma-separated list of x y coordinates
class KeepoutWidget : public QWidget
{
public:
  KeepoutWidget(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    auto layout = new QVBoxLayout(this);
    // allow entering comma-separated x,y points or pairs
    input_ = new QLineEdit(this);
    QPushButton *send = new QPushButton("Publish", this);
    layout->addWidget(input_);
    layout->addWidget(send);
    connect(send, &QPushButton::clicked, this, [this]() { onSend(); });
  }

private:
  void onSend()
  {
    // parse input and publish
    auto text = input_->text().toStdString();
    // simple parser: expect "x1,y1;x2,y2;..." coordinate list
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "map";
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ';')) {
      double x = 0.0, y = 0.0;
      if (std::sscanf(token.c_str(), "%lf,%lf", &x, &y) == 2) {
        geometry_msgs::msg::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = 0.0;
        msg.poses.push_back(p);
      }
    }
    if (publisher_) {
      publisher_->publish(msg);
    }
  }

public:
  void setPublisher(rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub)
  {
    publisher_ = pub;
  }

private:
  rclcpp::Node::SharedPtr node_;
  QLineEdit *input_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};

// simple graphics view that can load a map image and allow polygon drawing
class MapEditor : public QGraphicsView
{
  Q_OBJECT
public:
  MapEditor(QWidget *parent = nullptr)
  : QGraphicsView(parent), scene_(new QGraphicsScene(this)), resolution_(0.0)
  {
    setScene(scene_);
    setMouseTracking(true);
    drawing_ = false;
  }

  bool loadMap(const QString &image_path, double resolution, double origin_x, double origin_y)
  {
    QImage img(image_path);
    if (img.isNull()) return false;
    scene_->clear();
    scene_->addPixmap(QPixmap::fromImage(img));
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    polygon_.clear();
    return true;
  }

  std::vector<geometry_msgs::msg::Pose> getPolygonWorld() const
  {
    std::vector<geometry_msgs::msg::Pose> out;
    QPolygonF pts = polygon_;
    for (auto &p : pts) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = origin_x_ + p.x() * resolution_;
      pose.position.y = origin_y_ + (scene_->height() - p.y()) * resolution_;
      pose.position.z = 0.0;
      out.push_back(pose);
    }
    return out;
  }

protected:
  void mousePressEvent(QMouseEvent *event) override
  {
    if (event->button() == Qt::LeftButton) {
      drawing_ = true;
      startPt_ = mapToScene(event->pos());
      polygon_.append(startPt_);
    }
  }
  void mouseMoveEvent(QMouseEvent *event) override
  {
    if (drawing_) {
      QPointF pt = mapToScene(event->pos());
      polygon_.append(pt);
      redrawPolygon();
    }
  }
  void mouseReleaseEvent(QMouseEvent *event) override
  {
    if (drawing_ && event->button() == Qt::LeftButton) {
      drawing_ = false;
    }
  }

  void redrawPolygon()
  {
    if (polyItem_) scene_->removeItem(polyItem_);
    polyItem_ = scene_->addPolygon(polygon_, QPen(Qt::red));
  }

private:
  QGraphicsScene *scene_;
  QGraphicsPolygonItem *polyItem_ = nullptr;
  QPolygonF polygon_;
  bool drawing_;
  QPointF startPt_;
  double resolution_;
  double origin_x_, origin_y_;
};

class KeepoutEditor : public rclcpp::Node
{
public:
  KeepoutEditor()
  : Node("keepout_editor")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "prohibition_areas", 10);
    // create Qt application in separate thread or integrate with rclcpp
    // for simplicity run Qt event loop after spinning
  }

  void loadFromDatabase(const std::string & db_path, const std::string & table);
  void saveToDatabase(const std::string & db_path, const std::string & table);

  void run()
  {
    int argc = 0;
    QApplication app(argc, nullptr);
    KeepoutWidget widget(shared_from_this());
    widget.setPublisher(publisher_);
    // build additional UI around editor
    QWidget mainwin;
    auto layout = new QVBoxLayout(&mainwin);
    layout->addWidget(&widget);
    // DB controls
    auto hbox = new QHBoxLayout();
    dbPathEdit_ = new QLineEdit(&mainwin);
    dbPathEdit_->setPlaceholderText("Database path");
    tableEdit_ = new QLineEdit(&mainwin);
    tableEdit_->setPlaceholderText("Table name");
    QPushButton *browse = new QPushButton("Browse", &mainwin);
    QPushButton *load = new QPushButton("Load DB", &mainwin);
    hbox->addWidget(dbPathEdit_);
    hbox->addWidget(browse);
    hbox->addWidget(tableEdit_);
    hbox->addWidget(load);
    layout->addLayout(hbox);
    textView_ = new QPlainTextEdit(&mainwin);
    layout->addWidget(textView_);
    QObject::connect(browse, &QPushButton::clicked, this, [this](bool) {
      QString file = QFileDialog::getOpenFileName(&mainwin, "Select DB file");
      if (!file.isEmpty()) dbPathEdit_->setText(file);
    });
    QObject::connect(load, &QPushButton::clicked, this, [this](bool) {
      loadFromDatabase(dbPathEdit_->text().toStdString(), tableEdit_->text().toStdString());
    });
    mainwin.show();
    widget.show();
    app.exec();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  QLineEdit *dbPathEdit_;
  QLineEdit *tableEdit_;
  QPlainTextEdit *textView_;
};

}  // namespace filter_keepout

#endif  // FILTER_KEEPOUT__KEEPOUT_EDITOR_HPP_
