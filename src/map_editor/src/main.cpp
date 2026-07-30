#include <QApplication>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QToolBar>
#include <QAction>
#include <QSlider>
#include <QLabel>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QFile>
#include <QTextStream>
#include <QToolButton>
#include <QIcon>
#include <QDockWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QSpinBox>
#include <QTextEdit>
#include <QTimer>
#include <QDebug>
#include <QScrollBar>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <thread>
#include <atomic>
#include <mutex>

Q_DECLARE_METATYPE(QTextCursor)

#include <vector>
#include <tuple>
#include <cmath>
#include <functional>
#include <algorithm>

// Forward declaration
class ImageView;
class ControlPoint;
class SpeedPlot;

// Custom speed plot widget
class SpeedPlot : public QWidget
{
    Q_OBJECT
public:
    SpeedPlot(QWidget *parent = nullptr) : QWidget(parent)
    {
        setMinimumHeight(200);
        setMinimumWidth(300);
    }

    void addData(double time, double linear_x, double linear_y, double angular_z)
    {
        m_times.push_back(time);
        m_linearX.push_back(linear_x);
        m_linearY.push_back(linear_y);
        m_angularZ.push_back(angular_z);
        if (m_times.size() > 1000) { // Keep last 1000 points
            m_times.erase(m_times.begin());
            m_linearX.erase(m_linearX.begin());
            m_linearY.erase(m_linearY.begin());
            m_angularZ.erase(m_angularZ.begin());
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), Qt::white);
        painter.setPen(QPen(Qt::black, 1));
        painter.drawRect(rect());

        if (m_times.empty()) return;

        double minTime = *std::min_element(m_times.begin(), m_times.end());
        double maxTime = *std::max_element(m_times.begin(), m_times.end());
        double minSpeed = std::min({*std::min_element(m_linearX.begin(), m_linearX.end()),
                                    *std::min_element(m_linearY.begin(), m_linearY.end()),
                                    *std::min_element(m_angularZ.begin(), m_angularZ.end())});
        double maxSpeed = std::max({*std::max_element(m_linearX.begin(), m_linearX.end()),
                                    *std::max_element(m_linearY.begin(), m_linearY.end()),
                                    *std::max_element(m_angularZ.begin(), m_angularZ.end())});
        double rangeTime = maxTime - minTime;
        double rangeSpeed = maxSpeed - minSpeed;
        if (rangeTime == 0) rangeTime = 1;
        if (rangeSpeed == 0) rangeSpeed = 1;

        // Draw grid
        painter.setPen(QPen(Qt::lightGray, 1, Qt::DotLine));
        for (int i = 0; i <= 10; ++i) {
            int x = i * width() / 10;
            painter.drawLine(x, 0, x, height());
            int y = i * height() / 10;
            painter.drawLine(0, y, width(), y);
        }

        // Draw axes
        painter.setPen(QPen(Qt::black, 2));
        painter.drawLine(0, height()/2, width(), height()/2); // x-axis (time)
        painter.drawLine(width()/2, 0, width()/2, height()); // y-axis (speed)

        // Labels
        painter.setPen(QPen(Qt::black, 1));
        painter.drawText(width() - 50, height()/2 + 15, "Time");
        painter.drawText(width()/2 + 5, 15, "Speed");
        painter.drawText(width()/2 - 10, height() - 5, "0");

        // Draw linear x (blue)
        painter.setPen(QPen(Qt::blue, 2));
        for (size_t i = 1; i < m_times.size(); ++i) {
            int x1 = ((m_times[i-1] - minTime) / rangeTime) * width();
            int y1 = height() - ((m_linearX[i-1] - minSpeed) / rangeSpeed) * height();
            int x2 = ((m_times[i] - minTime) / rangeTime) * width();
            int y2 = height() - ((m_linearX[i] - minSpeed) / rangeSpeed) * height();
            painter.drawLine(x1, y1, x2, y2);
        }

        // Draw linear y (green)
        painter.setPen(QPen(Qt::green, 2));
        for (size_t i = 1; i < m_times.size(); ++i) {
            int x1 = ((m_times[i-1] - minTime) / rangeTime) * width();
            int y1 = height() - ((m_linearY[i-1] - minSpeed) / rangeSpeed) * height();
            int x2 = ((m_times[i] - minTime) / rangeTime) * width();
            int y2 = height() - ((m_linearY[i] - minSpeed) / rangeSpeed) * height();
            painter.drawLine(x1, y1, x2, y2);
        }

        // Draw angular z (red)
        painter.setPen(QPen(Qt::red, 2));
        for (size_t i = 1; i < m_times.size(); ++i) {
            int x1 = ((m_times[i-1] - minTime) / rangeTime) * width();
            int y1 = height() - ((m_angularZ[i-1] - minSpeed) / rangeSpeed) * height();
            int x2 = ((m_times[i] - minTime) / rangeTime) * width();
            int y2 = height() - ((m_angularZ[i] - minSpeed) / rangeSpeed) * height();
            painter.drawLine(x1, y1, x2, y2);
        }

        // Legend
        painter.setPen(QPen(Qt::black, 1));
        painter.drawText(10, 20, "Linear X (blue)");
        painter.drawText(10, 40, "Linear Y (green)");
        painter.drawText(10, 60, "Angular Z (red)");
    }

private:
    std::vector<double> m_times, m_linearX, m_linearY, m_angularZ;
};

// Custom control point item for draggable points
class ControlPoint : public QGraphicsEllipseItem
{
public:
    ControlPoint(const QPointF &pos, int index, std::function<void(int, const QPointF&)> callback)
        : QGraphicsEllipseItem(-10, -10, 20, 20), m_index(index), m_callback(callback)
    {
        setPos(pos);
        setBrush(QBrush(QColor(0, 255, 0, 128)));
        setPen(QPen(QColor(0, 255, 0)));
        setZValue(600);
    }

    int index() const { return m_index; }

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override
    {
        // Not used, manual dragging
        return QGraphicsEllipseItem::itemChange(change, value);
    }

private:
    int m_index;
    std::function<void(int, const QPointF&)> m_callback;
};

// Simple helper: load PGM (binary P5) into QImage (Format_Grayscale8)
static bool loadPGM(const QString &path, QImage &out)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
        return false;
    QByteArray data = f.readAll();
    // parse header
    int idx = 0;
    auto skipWs = [&](void){ while (idx < data.size() && (data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t')) ++idx; };
    // skip any leading whitespace
    while (idx < data.size() && (data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t')) ++idx;
    // magic (P5 = binary, P2 = ASCII)
    if (idx + 1 >= data.size()) return false;
    if (data[idx] != 'P') return false;
    char magic = data[idx+1];
    if (magic != '5' && magic != '2') return false;
    idx += 2;
    // skip whitespace and comments
    auto readToken = [&](QByteArray &outTok)->bool {
        outTok.clear();
        // skip whitespace and comments
        while (idx < data.size()) {
            if (data[idx] == '#') {
                // skip until newline
                while (idx < data.size() && data[idx] != '\n') ++idx;
                continue;
            }
            if (data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t') { ++idx; continue; }
            break;
        }
        if (idx >= data.size()) return false;
        while (idx < data.size() && !(data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t')) {
            outTok.append(data[idx]); ++idx;
        }
        return true;
    };
    QByteArray tok;
    bool ok;
    if (!readToken(tok)) return false; // width
    int w = QString(tok).toInt(&ok);
    if (!ok) return false;
    if (!readToken(tok)) return false; // height
    int h = QString(tok).toInt(&ok);
    if (!ok) return false;
    if (!readToken(tok)) return false; // maxval
    int maxv = QString(tok).toInt(&ok);
    if (!ok) return false;
    // idx currently at the position after maxval token; skip a single whitespace
    if (idx < data.size() && (data[idx] == '\n' || data[idx] == '\r' || data[idx] == ' ' || data[idx] == '\t')) ++idx;

    out = QImage(w, h, QImage::Format_Grayscale8);

    if (magic == '5') {
        // binary P5
        int samples = w * h;
        int bytes_per_sample = (maxv < 256) ? 1 : 2;
        qint64 expected = qint64(samples) * bytes_per_sample;
        if (data.size() - idx < expected) return false;
        const uchar *ptr = reinterpret_cast<const uchar*>(data.constData() + idx);
        for (int i = 0; i < samples; ++i) {
            int sample = 0;
            if (bytes_per_sample == 1) {
                sample = ptr[i];
            } else {
                // MSB first
                sample = (ptr[2*i] << 8) | ptr[2*i + 1];
            }
            // scale to 0..255 if necessary
            int scaled = (maxv <= 255) ? sample : int((sample * 255.0) / maxv + 0.5);
            int y = i / w;
            int x = i % w;
            uchar *line = out.scanLine(y);
            line[x] = static_cast<uchar>(qBound(0, scaled, 255));
        }
        return true;
    } else {
        // ASCII P2
        // read w*h integer tokens
        int count = 0;
        QByteArray tok2;
        for (int i = 0; i < w*h; ++i) {
            if (!readToken(tok2)) return false;
            int v = QString(tok2).toInt(&ok);
            if (!ok) return false;
            // scale if maxv > 255
            if (maxv > 255) {
                v = int((v * 255.0) / maxv + 0.5);
            }
            int y = i / w;
            int x = i % w;
            uchar *line = out.scanLine(y);
            line[x] = static_cast<uchar>(qBound(0, v, 255));
            ++count;
        }
        return count == w*h;
    }
}

// Save QImage as PGM (binary P5)
static bool savePGM(const QString &path, const QImage &img)
{
    if (img.format() != QImage::Format_Grayscale8) {
        QImage conv = img.convertToFormat(QImage::Format_Grayscale8);
        return savePGM(path, conv);
    }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) return false;
    QString header = QString("P5\n%1 %2\n255\n").arg(img.width()).arg(img.height());
    f.write(header.toUtf8());
    for (int y = 0; y < img.height(); ++y) {
        const uchar *line = img.constScanLine(y);
        f.write(reinterpret_cast<const char*>(line), img.width());
    }
    f.close();
    return true;
}

class ImageView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit ImageView(QWidget *parent=nullptr)
        : QGraphicsView(parent)
        , m_scene(new QGraphicsScene(this))
        , m_pixmapItem(nullptr)
        , m_brushItem(nullptr)
        , m_painting(false)
        , m_brushRadius(8)
        , m_drawValue(254)
        , m_maxUndo(10)
        , m_moveToggled(false)
        , m_previewEnabled(true)
    {
        setScene(m_scene);
        setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
        setDragMode(QGraphicsView::NoDrag);
        setInteractive(true);
        setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

        m_brushItem = m_scene->addEllipse(0,0,10,10, QPen(QColor(80,80,80,200)), QBrush(QColor(120,120,120,80)));
        m_brushItem->setZValue(1000);
        m_brushItem->setVisible(false);
    }

    void setImage(const QImage &img)
    {
        m_image = img.convertToFormat(QImage::Format_Grayscale8);
        m_undoStack.clear();
        m_undoStack.push_back(m_image);
        updatePixmap();
    }

    void setBrushRadius(int r) { m_brushRadius = qMax(1, r); }
    void setMoveMode(bool on) {
        m_moveToggled = on;
        if (m_moveToggled) {
            setDragMode(QGraphicsView::ScrollHandDrag);
            setCursor(Qt::OpenHandCursor);
            if (m_brushItem) m_brushItem->setVisible(false);
        } else {
            setDragMode(QGraphicsView::NoDrag);
            setCursor(Qt::ArrowCursor);
        }
    }

    // Path drawing helpers
    void setDrawMode(bool on) { m_drawMode = on; if (m_drawMode) { clearControlPoints(); } else { clearPathItem(); } }
    bool isDrawMode() const { return m_drawMode; }
    void addControlPoint(const QPointF &p) { 
        m_endPoints.push_back(p); 
        // Add a fixed endpoint item (green circle)
        QGraphicsEllipseItem *endItem = m_scene->addEllipse(p.x()-3, p.y()-3, 6, 6, QPen(QColor(0,255,0)), QBrush(QColor(0,255,0,200)));
        endItem->setZValue(550);
        m_endItems.push_back(endItem);
        // If at least 2 endpoints, add a draggable control point in the middle
        if (m_endPoints.size() >= 2) {
            QPointF mid = (m_endPoints[m_endPoints.size()-2] + m_endPoints.back()) * 0.5;
            m_controlPoints.push_back(mid);
            ControlPoint *cp = new ControlPoint(mid, m_controlPoints.size()-1, [this](int index, const QPointF &pos) { updateControlPoint(index, pos); });
            m_scene->addItem(cp);
            m_controlItems.push_back(cp);
        }
        m_previewEnabled = true; // enable preview after adding point
        updatePathPreview(); 
    }
    void clearControlPoints() { 
        for (auto *item : m_endItems) { m_scene->removeItem(item); delete item; } m_endItems.clear();
        for (auto *item : m_controlItems) { m_scene->removeItem(item); delete item; } m_controlItems.clear(); 
        m_endPoints.clear(); m_controlPoints.clear(); clearPathItem(); 
    }
    void updateControlPoint(int index, const QPointF &pos) { if (index >= 0 && index < (int)m_controlPoints.size()) { m_controlPoints[index] = pos; updatePathPreview(); } }
    // return sampled scene points along the current path
    std::vector<QPointF> samplePath(int samplesPerSegment = 100) const {
        std::vector<QPointF> outpts;
        int n = (int)m_endPoints.size();
        if (n < 2) return outpts;
        for (int i = 0; i < n-1; ++i) {
            QPointF p0 = m_endPoints[i];
            QPointF p1 = m_endPoints[i+1];
            QPointF c = m_controlPoints[i];
            for (int j = 0; j <= samplesPerSegment; ++j) {
                double t = double(j) / double(samplesPerSegment);
                double u = 1.0 - t;
                QPointF pt = u*u * p0 + 2*u*t * c + t*t * p1;
                outpts.push_back(pt);
            }
        }
        return outpts;
    }

    // Draw a persistent red path from sampled points
    void drawPersistentPath(const std::vector<QPointF> &pts) {
        if (m_pathItem) { m_scene->removeItem(m_pathItem); delete m_pathItem; m_pathItem = nullptr; }
        if (pts.empty()) return;
        m_currentPathPoints = pts;  // Store for redrawing
        if (m_showPath) {
            QPainterPath path;
            path.moveTo(pts[0]);
            for (size_t i = 1; i < pts.size(); ++i) path.lineTo(pts[i]);
            m_pathItem = m_scene->addPath(path, QPen(QColor(200,20,20), 1));
            m_pathItem->setZValue(500);
        }
        // Also draw points for density visualization
        if (m_showPoints) {
            for (size_t i = 0; i < pts.size(); i += m_pointDensity) {
                const QPointF &pt = pts[i];
                QGraphicsEllipseItem *pointItem = m_scene->addEllipse(pt.x()-2, pt.y()-2, 4, 4, QPen(QColor(20,20,200)), QBrush(QColor(20,20,200,150)));
                pointItem->setZValue(450);
                m_pathPoints.push_back(pointItem);
            }
        }
        // Draw orientation arrows
        if (m_showArrows) {
            for (size_t i = 0; i < pts.size(); i += m_arrowDensity) {
                const QPointF &pt = pts[i];
                double yaw = 0.0;
                if (i < pts.size() - 1) {
                    QPointF next = pts[i+1];
                    double dx = next.x() - pt.x();
                    double dy = next.y() - pt.y();
                    yaw = atan2(dy, dx);
                } else if (i > 0) {
                    QPointF prev = pts[i-1];
                    double dx = pt.x() - prev.x();
                    double dy = pt.y() - prev.y();
                    yaw = atan2(dy, dx);
                }
                // Draw arrow
                double arrowLength = 8.0;
                double arrowWidth = 4.0;
                QPointF tip = pt + QPointF(cos(yaw) * arrowLength, sin(yaw) * arrowLength);
                QPointF left = pt + QPointF(cos(yaw + M_PI/2) * arrowWidth, sin(yaw + M_PI/2) * arrowWidth);
                QPointF right = pt + QPointF(cos(yaw - M_PI/2) * arrowWidth, sin(yaw - M_PI/2) * arrowWidth);
                QPolygonF arrowPoly;
                arrowPoly << tip << left << right;
                QGraphicsPolygonItem *arrowItem = m_scene->addPolygon(arrowPoly, QPen(QColor(255,0,0)), QBrush(QColor(255,0,0,200)));
                arrowItem->setZValue(460);
                m_pathArrows.push_back(arrowItem);
            }
        }
    }

    void clearPathItem() {
        if (m_pathItem) { m_scene->removeItem(m_pathItem); delete m_pathItem; m_pathItem = nullptr; }
        if (m_previewItem) { m_scene->removeItem(m_previewItem); delete m_previewItem; m_previewItem = nullptr; }
        for (auto *item : m_pathPoints) { m_scene->removeItem(item); delete item; }
        m_pathPoints.clear();
        for (auto *item : m_pathArrows) { m_scene->removeItem(item); delete item; }
        m_pathArrows.clear();
    }

    void setMapParams(double resolution, double origin_x, double origin_y) {
        m_resolution = resolution;
        m_origin_x = origin_x;
        m_origin_y = origin_y;
    }

    void setShowPath(bool show) { m_showPath = show; redrawPath(); }
    void setShowPoints(bool show) { m_showPoints = show; redrawPath(); }
    void setShowArrows(bool show) { m_showArrows = show; redrawPath(); }
    void setPointDensity(int density) { m_pointDensity = density; redrawPath(); }
    void setArrowDensity(int density) { m_arrowDensity = density; redrawPath(); }

    void updateRobotPose(double x, double y, double yaw) {
        if (m_image.isNull()) return; // No map loaded
        // Convert world to pixel
        double px = (x - m_origin_x) / m_resolution;
        double py = m_image.height() - 1 - (y - m_origin_y) / m_resolution;

        if (!m_robotPoseItem) {
            m_robotPoseItem = m_scene->addEllipse(px-5, py-5, 10, 10, QPen(QColor(255,0,0)), QBrush(QColor(255,0,0,200)));
            m_robotPoseItem->setZValue(700);
        } else {
            m_robotPoseItem->setRect(px-5, py-5, 10, 10);
        }

        // Arrow
        double arrowLength = 15.0;
        QPointF tip(px + cos(yaw) * arrowLength, py + sin(yaw) * arrowLength);
        QPointF left(px + cos(yaw + M_PI/2) * 5, py + sin(yaw + M_PI/2) * 5);
        QPointF right(px + cos(yaw - M_PI/2) * 5, py + sin(yaw - M_PI/2) * 5);
        QPolygonF arrowPoly;
        arrowPoly << tip << left << right;
        if (!m_robotArrowItem) {
            m_robotArrowItem = m_scene->addPolygon(arrowPoly, QPen(QColor(255,0,0)), QBrush(QColor(255,0,0,200)));
            m_robotArrowItem->setZValue(710);
        } else {
            m_robotArrowItem->setPolygon(arrowPoly);
        }
    }

    void clearRobotPose() {
        if (m_robotPoseItem) { m_scene->removeItem(m_robotPoseItem); delete m_robotPoseItem; m_robotPoseItem = nullptr; }
        if (m_robotArrowItem) { m_scene->removeItem(m_robotArrowItem); delete m_robotArrowItem; m_robotArrowItem = nullptr; }
    }

    void updateScan(const std::vector<QPointF> &points) {
        if (m_image.isNull()) return; // No map loaded
        clearScan();
        for (const auto &p : points) {
            double px = (p.x() - m_origin_x) / m_resolution;
            double py = m_image.height() - 1 - (p.y() - m_origin_y) / m_resolution;
            QGraphicsEllipseItem *item = m_scene->addEllipse(px-1, py-1, 2, 2, QPen(QColor(0,255,0)), QBrush(QColor(0,255,0,150)));
            item->setZValue(650);
            m_scanPoints.push_back(item);
        }
    }

    void clearScan() {
        for (auto *item : m_scanPoints) { m_scene->removeItem(item); delete item; }
        m_scanPoints.clear();
    }

    void updateObstacles(const std::vector<QPointF> &points) {
        if (m_image.isNull()) return; // No map loaded
        clearObstacles();
        for (const auto &p : points) {
            double px = (p.x() - m_origin_x) / m_resolution;
            double py = m_image.height() - 1 - (p.y() - m_origin_y) / m_resolution;
            QGraphicsEllipseItem *item = m_scene->addEllipse(px-2, py-2, 4, 4, QPen(QColor(255,255,0)), QBrush(QColor(255,255,0,150)));
            item->setZValue(650);
            m_obstaclePoints.push_back(item);
        }
    }

    void clearObstacles() {
        for (auto *item : m_obstaclePoints) { m_scene->removeItem(item); delete item; }
        m_obstaclePoints.clear();
    }

    void undo()
    {
        if (m_undoStack.size() < 2) return;
        m_undoStack.pop_back();
        m_image = m_undoStack.back();
        updatePixmap();
    }

private:
    void redrawPath() {
        clearPathItem();
        std::vector<QPointF> pts;
        if (m_drawMode) {
            pts = samplePath();
        } else if (!m_currentPathPoints.empty()) {
            pts = m_currentPathPoints;
        }
        if (!pts.empty()) {
            drawPersistentPath(pts);
        }
    }

protected:
    void mousePressEvent(QMouseEvent *event) override
    {
        setFocus();
        if (dragMode() == QGraphicsView::ScrollHandDrag) {
            setCursor(Qt::ClosedHandCursor);
            QGraphicsView::mousePressEvent(event);
            return;
        }
        if (m_drawMode && event->button() == Qt::LeftButton) {
            // add control point in scene coords
            QPointF scenePt = mapToScene(event->pos());
            // Check if clicking on an existing ControlPoint
            QGraphicsItem *item = m_scene->itemAt(scenePt, QTransform());
            ControlPoint *cp = dynamic_cast<ControlPoint*>(item);
            if (cp) {
                // Start dragging
                m_dragging = true;
                m_draggingIndex = cp->index();
                return;
            } else if (item == nullptr || dynamic_cast<QGraphicsEllipseItem*>(item) == nullptr || std::find(m_endItems.begin(), m_endItems.end(), dynamic_cast<QGraphicsEllipseItem*>(item)) == m_endItems.end()) {
                // Add new endpoint if not clicking on existing endpoint
                addControlPoint(scenePt);
                return;
            }
            // If clicking on endpoint, do nothing
        }
        if (m_drawMode && event->button() == Qt::RightButton) {
            // Disable preview on right click
            m_previewEnabled = false;
            updatePathPreview();
            return;
        }
        if (event->button() == Qt::LeftButton && !m_image.isNull() && !m_moveToggled) {
            m_painting = true;
            pushUndo();
            paintAtEvent(event->pos());
            if (m_brushItem) { m_brushItem->setVisible(true); updateBrushOverlay(event->pos()); }
            return;
        }
        QGraphicsView::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (dragMode() == QGraphicsView::ScrollHandDrag) {
            QGraphicsView::mouseMoveEvent(event);
            return;
        }
        if (m_drawMode) {
            if (m_dragging) {
                // Update dragged control point
                QPointF pos = mapToScene(event->pos());
                updateControlPoint(m_draggingIndex, pos);
                return;
            } else {
                // Show preview
                updatePathPreview(event->pos());
                return;
            }
        }
        if (m_painting && !m_image.isNull()) {
            paintAtEvent(event->pos());
            return;
        }
        // do not track brush when not painting
        QGraphicsView::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            if (m_dragging) {
                m_dragging = false;
                m_draggingIndex = -1;
                return;
            }
            if (dragMode() == QGraphicsView::ScrollHandDrag) {
                QGraphicsView::mouseReleaseEvent(event);
                if (m_moveToggled) setCursor(Qt::OpenHandCursor); else setCursor(Qt::ArrowCursor);
                return;
            }
            if (m_painting) {
                m_painting = false;
                if (m_brushItem) m_brushItem->setVisible(false);
                return;
            }
        }
        QGraphicsView::mouseReleaseEvent(event);
    }

    // Update preview path item from control points; optional viewPos to show last segment to cursor
    void updatePathPreview(const QPoint &viewPos = QPoint()) {
        if (!m_drawMode) return;
        // Draw the current path, and if viewPos and preview enabled, add a line to cursor
        std::vector<QPointF> pts = samplePath();
        if (m_previewEnabled && !viewPos.isNull() && !m_endPoints.empty()) {
            QPointF last = m_endPoints.back();
            QPointF cursor = mapToScene(viewPos);
            pts.push_back(last);
            pts.push_back(cursor);
        }
        if (m_previewItem) { m_scene->removeItem(m_previewItem); delete m_previewItem; m_previewItem = nullptr; }
        if (pts.empty()) return;
        QPainterPath p;
        p.moveTo(pts[0]);
        for (size_t i = 1; i < pts.size(); ++i) p.lineTo(pts[i]);
        m_previewItem = m_scene->addPath(p, QPen(QColor(100,100,255), 1, Qt::DashLine));
        m_previewItem->setZValue(400);
    }

    void wheelEvent(QWheelEvent *event) override
    {
        const int delta = event->angleDelta().y();
        qreal factor = (delta > 0) ? 1.15 : (1.0/1.15);
        scale(factor, factor);
    }

    void leaveEvent(QEvent *event) override
    {
        if (m_brushItem) m_brushItem->setVisible(false);
        QGraphicsView::leaveEvent(event);
    }

private:
    void updatePixmap()
    {
        if (m_image.isNull()) return;
        QPixmap pix = QPixmap::fromImage(m_image);
        if (!m_pixmapItem) m_pixmapItem = m_scene->addPixmap(pix);
        else m_pixmapItem->setPixmap(pix);
        m_scene->setSceneRect(0,0,m_image.width(), m_image.height());
    }

    void pushUndo()
    {
        if (m_image.isNull()) return;
        m_undoStack.push_back(m_image);
        if ((int)m_undoStack.size() > m_maxUndo) m_undoStack.erase(m_undoStack.begin());
    }

    void paintAtEvent(const QPoint &viewPos)
    {
        QPointF scenePt = mapToScene(viewPos);
        int x = int(scenePt.x());
        int y = int(scenePt.y());
        if (m_image.isNull()) return;
        int w = m_image.width();
        int h = m_image.height();
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        int r = m_brushRadius;
        QPainter p(&m_image);
        p.setCompositionMode(QPainter::CompositionMode_Source);
        QColor c(m_drawValue, m_drawValue, m_drawValue);
        p.setBrush(QBrush(c));
        p.setPen(Qt::NoPen);
        p.drawEllipse(QPoint(x,y), r, r);
        p.end();
        updatePixmap();
        // keep overlay in sync while painting
        updateBrushOverlay(viewPos);
    }

    void updateBrushOverlay(const QPoint &viewPos)
    {
        if (m_image.isNull() || !m_brushItem) return;
        QPointF scenePt = mapToScene(viewPos);
        double x = scenePt.x();
        double y = scenePt.y();
        if (x < 0 || y < 0 || x >= m_image.width() || y >= m_image.height()) { m_brushItem->setVisible(false); return; }
        int r = m_brushRadius;
        QRectF rect(x - r, y - r, 2*r, 2*r);
        m_brushItem->setRect(rect);
        m_brushItem->setVisible(true);
    }

private:
    QGraphicsScene *m_scene;
    QGraphicsPixmapItem *m_pixmapItem;
    QGraphicsEllipseItem *m_brushItem;
    QImage m_image;
    bool m_painting;
    int m_brushRadius;
    int m_drawValue;
    std::vector<QImage> m_undoStack;
    int m_maxUndo;
    bool m_moveToggled;
    // path drawing state
    bool m_drawMode = false;
    std::vector<QPointF> m_endPoints;
    std::vector<QGraphicsEllipseItem*> m_endItems;
    std::vector<QPointF> m_controlPoints;
    std::vector<ControlPoint*> m_controlItems;
    QGraphicsPathItem *m_previewItem = nullptr;
    QGraphicsPathItem *m_pathItem = nullptr;
    std::vector<QGraphicsEllipseItem*> m_pathPoints;
    std::vector<QGraphicsPolygonItem*> m_pathArrows;
    bool m_dragging = false;
    int m_draggingIndex = -1;
    bool m_previewEnabled;
    // path display options
    bool m_showPath = true;
    bool m_showPoints = true;
    bool m_showArrows = true;
    int m_pointDensity = 1;
    int m_arrowDensity = 10;
    // current path points for redrawing
    std::vector<QPointF> m_currentPathPoints;
    // ROS2 visualization
    QGraphicsEllipseItem *m_robotPoseItem = nullptr;
    QGraphicsPolygonItem *m_robotArrowItem = nullptr;
    std::vector<QGraphicsEllipseItem*> m_scanPoints;
    std::vector<QGraphicsEllipseItem*> m_obstaclePoints;
    double m_resolution = 1.0;
    double m_origin_x = 0.0;
    double m_origin_y = 0.0;
};

struct TopicInfo {
    QString name;
    QString type;
    bool subscribed = false;
    QPushButton *button = nullptr;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow()
    {
        // ROS2 node creation (init already done in main)
        m_node = rclcpp::Node::make_shared("map_editor_node");
        m_executor.add_node(m_node);
        m_executor_thread = std::thread([this]() { m_executor.spin(); });
        m_executor_thread.detach();

        setWindowTitle("PGM辅助工具");
        resize(1200,800);

        m_view = new ImageView(this);
        setCentralWidget(m_view);

        // Create dock widget for ROS2 topics
        m_dock = new QDockWidget("ROS2 话题订阅", this);
        m_dock->setAllowedAreas(Qt::RightDockWidgetArea);
        addDockWidget(Qt::RightDockWidgetArea, m_dock);

        QWidget *dockWidget = new QWidget();
        QVBoxLayout *dockLayout = new QVBoxLayout(dockWidget);

        m_topicTable = new QTableWidget();
        m_topicTable->setColumnCount(3);
        m_topicTable->setHorizontalHeaderLabels({"话题名", "类型", "操作"});
        m_topicTable->horizontalHeader()->setStretchLastSection(true);
        m_topicTable->setRowCount(3);

        // Add topics
        addTopicRow(0, "amcl_pose", "geometry_msgs/PoseWithCovarianceStamped");
        addTopicRow(1, "scan", "sensor_msgs/LaserScan");
        addTopicRow(2, "obstacles", "sensor_msgs/PointCloud2");

        dockLayout->addWidget(m_topicTable);

        // Path display options
        m_showPathCheck = new QCheckBox("显示路径");
        m_showPathCheck->setChecked(true);
        connect(m_showPathCheck, &QCheckBox::toggled, this, [this](bool checked){ m_view->setShowPath(checked); });
        dockLayout->addWidget(m_showPathCheck);

        m_showPointsCheck = new QCheckBox("显示点");
        m_showPointsCheck->setChecked(true);
        connect(m_showPointsCheck, &QCheckBox::toggled, this, [this](bool checked){ m_view->setShowPoints(checked); });
        dockLayout->addWidget(m_showPointsCheck);

        m_showArrowsCheck = new QCheckBox("显示箭头");
        m_showArrowsCheck->setChecked(true);
        connect(m_showArrowsCheck, &QCheckBox::toggled, this, [this](bool checked){ m_view->setShowArrows(checked); });
        dockLayout->addWidget(m_showArrowsCheck);

        QLabel *pointDensityLabel = new QLabel("点密度 (每N个点显示1个):");
        dockLayout->addWidget(pointDensityLabel);

        m_pointDensitySpin = new QSpinBox();
        m_pointDensitySpin->setRange(1, 100);
        m_pointDensitySpin->setValue(1);
        connect(m_pointDensitySpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int val){ m_view->setPointDensity(val); });
        dockLayout->addWidget(m_pointDensitySpin);

        QLabel *arrowDensityLabel = new QLabel("箭头密度 (每N个点显示1个):");
        dockLayout->addWidget(arrowDensityLabel);

        m_arrowDensitySpin = new QSpinBox();
        m_arrowDensitySpin->setRange(1, 100);
        m_arrowDensitySpin->setValue(10);
        connect(m_arrowDensitySpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int val){ m_view->setArrowDensity(val); });
        dockLayout->addWidget(m_arrowDensitySpin);

        // Log window
        QLabel *logLabel = new QLabel("日志:");
        dockLayout->addWidget(logLabel);
        m_logText = new QTextEdit();
        m_logText->setMaximumHeight(150);
        m_logText->setReadOnly(true);
        dockLayout->addWidget(m_logText);

        // Speed topic section
        QLabel *speedLabel = new QLabel("速度话题:");
        dockLayout->addWidget(speedLabel);
        QHBoxLayout *speedLayout = new QHBoxLayout();
        m_speedTopicEdit = new QLineEdit("cmd_vel");
        speedLayout->addWidget(m_speedTopicEdit);
        m_speedSubscribeBtn = new QPushButton("订阅");
        connect(m_speedSubscribeBtn, &QPushButton::clicked, this, &MainWindow::toggleSpeedSubscription);
        speedLayout->addWidget(m_speedSubscribeBtn);
        dockLayout->addLayout(speedLayout);

        // Speed plot
        QLabel *plotLabel = new QLabel("速度绘图 (线性:蓝, 角:红):");
        dockLayout->addWidget(plotLabel);
        m_speedPlot = new SpeedPlot();
        dockLayout->addWidget(m_speedPlot);

        m_dock->setWidget(dockWidget);

        QToolBar *toolbar = addToolBar("Tools");

        QAction *openAct = new QAction("打开", this);
        connect(openAct, &QAction::triggered, this, &MainWindow::onOpen);
        toolbar->addAction(openAct);

        QAction *saveAct = new QAction("保存", this);
        connect(saveAct, &QAction::triggered, this, &MainWindow::onSave);
        toolbar->addAction(saveAct);

        QAction *undoAct = new QAction("撤销", this);
        connect(undoAct, &QAction::triggered, m_view, &ImageView::undo);
        toolbar->addAction(undoAct);

        QAction *drawAct = new QAction("绘制路径", this);
        drawAct->setCheckable(true);
        connect(drawAct, &QAction::toggled, this, [this](bool on){
            if (m_view) m_view->setDrawMode(on);
            if (on) m_status->showMessage("绘制模式: 开启 (点击添加控制点). 点击保存路径完成绘制。");
            else m_status->showMessage("绘制模式: 关闭");
        });
        toolbar->addAction(drawAct);

        QAction *savePathAct = new QAction("路径保存", this);
        connect(savePathAct, &QAction::triggered, this, &MainWindow::onSavePath);
        toolbar->addAction(savePathAct);

        QAction *openPathAct = new QAction("打开路径", this);
        connect(openPathAct, &QAction::triggered, this, &MainWindow::onOpenPath);
        toolbar->addAction(openPathAct);

        m_moveAct = new QAction("移动", this);
        m_moveAct->setCheckable(true);
        connect(m_moveAct, &QAction::toggled, this, &MainWindow::onMoveToggled);
        toolbar->addAction(m_moveAct);

        toolbar->addSeparator();
        QLabel *brushLabel = new QLabel("尺寸调整");
        toolbar->addWidget(brushLabel);
        m_brushSlider = new QSlider(Qt::Horizontal);
        m_brushSlider->setMinimum(1);
        m_brushSlider->setMaximum(100);
        m_brushSlider->setValue(8);
        m_brushSlider->setFixedWidth(140);
        connect(m_brushSlider, &QSlider::valueChanged, this, &MainWindow::onBrushChanged);
        toolbar->addWidget(m_brushSlider);

        QWidget *spacer = new QWidget();
        spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        toolbar->addWidget(spacer);

        QToolButton *infoButton = new QToolButton();
        infoButton->setIcon(QIcon::fromTheme("help-about"));
        infoButton->setToolTip("左键拖拽：擦除 → 释放；滚轮：缩放；使用“移动”工具平移地图");
        toolbar->addWidget(infoButton);

        m_status = statusBar();

        // Connect signals to slots for thread-safe UI updates
        connect(this, &MainWindow::speedUpdated, this, &MainWindow::onSpeedUpdated);
        connect(this, &MainWindow::logUpdated, this, &MainWindow::onLogUpdated);
    }

signals:
    void speedUpdated(double time, double linear_x, double linear_y, double angular_z);
    void logUpdated(const QString &text);

private slots:
    void onSpeedUpdated(double time, double linear_x, double linear_y, double angular_z)
    {
        m_speedPlot->addData(time, linear_x, linear_y, angular_z);
        m_logText->append(QString("Speed: linear_x=%1, linear_y=%2, angular_z=%3").arg(linear_x, 0, 'f', 2).arg(linear_y, 0, 'f', 2).arg(angular_z, 0, 'f', 2));
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
    }

    void onLogUpdated(const QString &text)
    {
        m_logText->append(text);
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
    }

    void onOpen()
    {
        QString fname = QFileDialog::getOpenFileName(this, "打开PGM", QDir::currentPath(), "PGM Files (*.pgm);;All Files (*)");
        if (fname.isEmpty()) return;
        QImage img;
        if (!loadPGM(fname, img)) {
            QMessageBox::warning(this, "打开PGM", "无法打开PGM文件（仅支持二进制P5格式）");
            return;
        }
        m_view->clearPathItem();
        m_view->setImage(img);
        m_currentPath = fname;
        m_mapLoaded = true;
        m_status->showMessage(QString("已加载: %1").arg(fname));

        // Load YAML for map params
        QFileInfo fi(fname);
        QString yamlPath = fi.absolutePath() + "/" + fi.completeBaseName() + ".yaml";
        double resolution = 1.0;
        double origin_x = 0.0, origin_y = 0.0;
        if (QFile::exists(yamlPath)) {
            QFile yf(yamlPath);
            if (yf.open(QIODevice::ReadOnly)) {
                QTextStream in(&yf);
                while (!in.atEnd()) {
                    QString line = in.readLine().trimmed();
                    if (line.startsWith("resolution:")) {
                        auto parts = line.split(':');
                        if (parts.size() >= 2) resolution = parts[1].trimmed().toDouble();
                    } else if (line.startsWith("origin:")) {
                        int lb = line.indexOf('[');
                        int rb = line.indexOf(']');
                        if (lb >= 0 && rb > lb) {
                            QString inside = line.mid(lb+1, rb-lb-1);
                            QStringList nums = inside.split(',');
                            if (nums.size() >= 2) {
                                origin_x = nums[0].trimmed().toDouble();
                                origin_y = nums[1].trimmed().toDouble();
                            }
                        }
                    }
                }
                yf.close();
            }
        }
        m_view->setMapParams(resolution, origin_x, origin_y);
    }

    void onSave()
    {
        if (m_view == nullptr) return;
        QString fname = QFileDialog::getSaveFileName(this, "保存PGM", m_currentPath.isEmpty() ? QDir::currentPath() : m_currentPath, "PGM Files (*.pgm)");
        if (fname.isEmpty()) return;
        // ask view's image via friend method by saving from scene pixmap
        // For simplicity, we assume view->m_image exists; use a dynamic cast to access private? Instead we will grab current pixmap from scene.
        // Better: add a public method to ImageView to return the current QImage. We'll access via QObject property using Qt::direct connection: but simpler, add a lambda that uses a public method. For now, cast.
        // Use a signal-slot approach: call a public method via pointer.
        // (We declared ImageView::setImage/undo publicly; but not getter. We'll temporarily call grab of scene pixmap.)
        // To keep code simple, add a small workaround: render scene to image
        QImage out(m_view->scene()->sceneRect().size().toSize(), QImage::Format_Grayscale8);
        out.fill(255);
        QPainter p(&out);
        m_view->scene()->render(&p);
        p.end();
        if (!savePGM(fname, out)) {
            QMessageBox::critical(this, "保存错误", "无法保存PGM文件");
            return;
        }
        m_status->showMessage(QString("已保存: %1").arg(fname));
    }

    // Save drawn path: sample bezier, convert scene pixels -> world coords via YAML, write wheeltec_path file
    std::vector<std::tuple<double,double,double>> resamplePath(const std::vector<std::tuple<double,double,double>>& path, double target_distance) {
        std::vector<std::tuple<double,double,double>> resampled;
        if (path.empty()) return resampled;
        resampled.push_back(path[0]);
        for (size_t i = 1; i < path.size(); ++i) {
            double x1, y1, yaw1; std::tie(x1, y1, yaw1) = path[i-1];
            double x2, y2, yaw2; std::tie(x2, y2, yaw2) = path[i];
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist > 0) {
                int num_points = std::floor(dist / target_distance);
                for (int j = 1; j <= num_points; ++j) {
                    double t = j * target_distance / dist;
                    double x = x1 + t * dx;
                    double y = y1 + t * dy;
                    double yaw = yaw1 + t * (yaw2 - yaw1); // linear interpolation
                    resampled.emplace_back(x, y, yaw);
                }
            }
        }
        // Add the last point
        resampled.push_back(path.back());
        return resampled;
    }

    void onSavePath()
    {
        if (!m_view) return;
        // sample path from view with reduced density
        auto pts = m_view->samplePath(10); // reduced from 200 to 10 for lower density
        if (pts.empty()) {
            QMessageBox::warning(this, "保存路径", "没有绘制路径。");
            return;
        }
        // find yaml for current map image (same directory, same base name)
        if (m_currentPath.isEmpty()) {
            QMessageBox::warning(this, "保存路径", "没有加载地图以推断YAML文件。");
            return;
        }
        QFileInfo fi(m_currentPath);
        QString yamlPath = fi.absolutePath() + "/" + fi.completeBaseName() + ".yaml";
        double resolution = 0.0;
        double origin_x = 0.0, origin_y = 0.0; // origin[0], origin[1]
        bool gotYaml = false;
        if (QFile::exists(yamlPath)) {
            QFile yf(yamlPath);
            if (yf.open(QIODevice::ReadOnly)) {
                QTextStream in(&yf);
                while (!in.atEnd()) {
                    QString line = in.readLine().trimmed();
                    if (line.startsWith("resolution:")) {
                        auto parts = line.split(':');
                        if (parts.size() >= 2) resolution = parts[1].trimmed().toDouble();
                    } else if (line.startsWith("origin:")) {
                        // origin: [0.295, -60.4, 0]
                        int lb = line.indexOf('[');
                        int rb = line.indexOf(']');
                        if (lb >= 0 && rb > lb) {
                            QString inside = line.mid(lb+1, rb-lb-1);
                            QStringList nums = inside.split(',');
                            if (nums.size() >= 2) {
                                origin_x = nums[0].trimmed().toDouble();
                                origin_y = nums[1].trimmed().toDouble();
                                gotYaml = true;
                            }
                        }
                    }
                }
                yf.close();
            }
        }
        if (!gotYaml) {
            QMessageBox::warning(this, "保存路径", QString("无法找到/解析YAML: %1\n使用默认原点/分辨率 (0,0,1.0)").arg(yamlPath));
            resolution = (resolution <= 0.0) ? 1.0 : resolution;
            origin_x = 0.0; origin_y = 0.0;
        }

        // convert scene pts to image pixel coordinates: scene coords start at (0,0) top-left matching image
        int imgH = m_view->scene()->height();
        // sample and compute world coords
        std::vector<std::tuple<double,double,double>> out; // x,y,yaw
        for (size_t i = 0; i+1 < pts.size(); ++i) {
            QPointF p = pts[i];
            QPointF p2 = pts[i+1];
            int px = int(std::round(p.x()));
            int py = int(std::round(p.y()));
            int px2 = int(std::round(p2.x()));
            int py2 = int(std::round(p2.y()));
            // convert pixel -> world (assume origin corresponds to map bottom-left)
            double wx = origin_x + px * resolution;
            double wy = origin_y + ( (double)(imgH - 1 - py) ) * resolution;
            double wx2 = origin_x + px2 * resolution;
            double wy2 = origin_y + ( (double)(imgH - 1 - py2) ) * resolution;
            double yaw = std::atan2(wy2 - wy, wx2 - wx);
            out.emplace_back(wx, wy, yaw);
        }
        // last point replicate last yaw
        if (out.empty()) {
            QMessageBox::warning(this, "保存路径", "没有足够的点来计算路径。");
            return;
        }
        out.push_back(out.back());

        // Resample path to 0.05m density
        double target_distance = 0.05;
        auto resampled_out = resamplePath(out, target_distance);

        QString saveName = QFileDialog::getSaveFileName(this, "保存路径为", fi.absolutePath() + "/cdf_path", "Path Files (*)");
        if (saveName.isEmpty()) return;
        QFile of(saveName);
        if (!of.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "保存路径", "无法打开输出文件进行写入");
            return;
        }
        QTextStream outStream(&of);
        for (auto &tup : resampled_out) {
            double x,y,yaw; std::tie(x,y,yaw) = tup;
            outStream << QString::number(x,'f',6) << " " << QString::number(y,'f',6) << " " << QString::number(yaw,'f',6) << "\n";
        }
        outStream << "EOP\n";
        of.close();
        m_status->showMessage(QString("已保存路径: %1").arg(saveName));
        // draw persistent path using the saved points (converted back to scene coords)
        std::vector<QPointF> scenePts;
        for (auto &tup : resampled_out) {
            double x,y,yaw; std::tie(x,y,yaw) = tup;
            double px = (x - origin_x) / resolution;
            double py = imgH - 1 - (y - origin_y) / resolution;
            scenePts.emplace_back(px, py);
        }
        m_view->drawPersistentPath(scenePts);
    }

    void onOpenPath()
    {
        if (!m_view || m_currentPath.isEmpty()) {
            QMessageBox::warning(this, "打开路径", "没有加载地图。");
            return;
        }
        // Clear previous path
        m_view->clearPathItem();
        // find yaml
        QFileInfo fi(m_currentPath);
        QString yamlPath = fi.absolutePath() + "/" + fi.completeBaseName() + ".yaml";
        double resolution = 1.0;
        double origin_x = 0.0, origin_y = 0.0;
        bool gotYaml = false;
        if (QFile::exists(yamlPath)) {
            QFile yf(yamlPath);
            if (yf.open(QIODevice::ReadOnly)) {
                QTextStream in(&yf);
                while (!in.atEnd()) {
                    QString line = in.readLine().trimmed();
                    if (line.startsWith("resolution:")) {
                        auto parts = line.split(':');
                        if (parts.size() >= 2) resolution = parts[1].trimmed().toDouble();
                    } else if (line.startsWith("origin:")) {
                        int lb = line.indexOf('[');
                        int rb = line.indexOf(']');
                        if (lb >= 0 && rb > lb) {
                            QString inside = line.mid(lb+1, rb-lb-1);
                            QStringList nums = inside.split(',');
                            if (nums.size() >= 2) {
                                origin_x = nums[0].trimmed().toDouble();
                                origin_y = nums[1].trimmed().toDouble();
                                gotYaml = true;
                            }
                        }
                    }
                }
                yf.close();
            }
        }
        if (!gotYaml) {
            QMessageBox::warning(this, "打开路径", QString("无法找到/解析YAML: %1\n使用默认原点/分辨率 (0,0,1.0)").arg(yamlPath));
        }

        QString fname = QFileDialog::getOpenFileName(this, "打开路径", fi.absolutePath(), "Path Files (*)");
        if (fname.isEmpty()) return;
        QFile f(fname);
        if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "打开路径", "无法打开路径文件");
            return;
        }
        QTextStream in(&f);
        std::vector<QPointF> pts;
        int imgH = m_view->scene()->height();
        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();
            if (line == "EOP") break;
            QStringList parts = line.split(' ', Qt::SkipEmptyParts);
            if (parts.size() >= 2) {
                double x = parts[0].toDouble();
                double y = parts[1].toDouble();
                // convert world to pixel
                double px = (x - origin_x) / resolution;
                double py = imgH - 1 - (y - origin_y) / resolution;
                pts.emplace_back(px, py);
            }
        }
        f.close();
        if (pts.empty()) {
            QMessageBox::warning(this, "打开路径", "路径点文件没有点！");
            return;
        }
        m_view->drawPersistentPath(pts);
        m_status->showMessage(QString("加载路径中: %1").arg(fname));
    }

    void onBrushChanged(int val)
    {
        if (m_view) m_view->setBrushRadius(val);
        m_status->showMessage(QString("调整尺寸: %1").arg(val));
    }

    void onMoveToggled(bool on)
    {
        if (m_view) m_view->setMoveMode(on);
        if (on) m_status->showMessage("Move mode: ON"); else m_status->showMessage("Move mode: OFF");
    }

    void addTopicRow(int row, const QString &topic, const QString &type)
    {
        m_topicTable->setItem(row, 0, new QTableWidgetItem(topic));
        m_topicTable->setItem(row, 1, new QTableWidgetItem(type));

        QPushButton *btn = new QPushButton("订阅");
        connect(btn, &QPushButton::clicked, this, [this, row, topic, type, btn]() {
            toggleSubscription(row, topic, type, btn);
        });
        m_topicTable->setCellWidget(row, 2, btn);

        TopicInfo info;
        info.name = topic;
        info.type = type;
        info.button = btn;
        m_topics.push_back(info);
    }

    void toggleSubscription(int row, const QString &topic, const QString &type, QPushButton *btn)
    {
        if (m_topics[row].subscribed) {
            // Unsubscribe
            unsubscribeTopic(row);
            btn->setText("订阅");
            m_topics[row].subscribed = false;
        } else {
            // Subscribe
            subscribeTopic(row, topic, type);
            btn->setText("取消订阅");
            m_topics[row].subscribed = true;
        }
    }

    void subscribeTopic(int row, const QString &topic, const QString &type)
    {
        rclcpp::QoS qos(10); // Default QoS
        if (type == "geometry_msgs/PoseWithCovarianceStamped") {
            qos = rclcpp::QoS(10).best_effort(); // AMCL uses best_effort
        } else {
            qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)); // RELIABLE for sensors
        }
        if (type == "geometry_msgs/PoseWithCovarianceStamped") {
            m_pose_sub = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                topic.toStdString(), qos,
                [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    updateRobotPose(msg);
                });
        } else if (type == "sensor_msgs/LaserScan") {
            m_scan_sub = m_node->create_subscription<sensor_msgs::msg::LaserScan>(
                topic.toStdString(), qos,
                [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    updateScan(msg);
                });
        } else if (type == "sensor_msgs/PointCloud2") {
            m_obstacles_sub = m_node->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic.toStdString(), qos,
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    updateObstacles(msg);
                });
        }
        m_logText->append(QString("Subscribed to topic: %1 (%2) with QoS").arg(topic).arg(type));
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
    }

    void unsubscribeTopic(int row)
    {
        if (m_topics[row].name == "amcl_pose") {
            m_pose_sub.reset();
            m_view->clearRobotPose();
        } else if (m_topics[row].name == "scan") {
            m_scan_sub.reset();
            m_view->clearScan();
        } else if (m_topics[row].name == "obstacles") {
            m_obstacles_sub.reset();
            m_view->clearObstacles();
        }
        m_logText->append(QString("Unsubscribed from topic: %1").arg(m_topics[row].name));
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
    }

    void updateRobotPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!m_mapLoaded) {
            m_logText->append("Robot pose received but no map loaded");
            m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
            return;
        }
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        m_robot_x = x;
        m_robot_y = y;
        m_robot_yaw = yaw;
        qDebug() << "Robot pose received:" << x << y << yaw;
        m_logText->append(QString("Robot pose: x=%1, y=%2, yaw=%3").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw, 0, 'f', 2));
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
        m_view->updateRobotPose(x, y, yaw);
    }

    void updateScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<QPointF> points;
        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max) {
                double lx = r * cos(angle);
                double ly = r * sin(angle);
                // Transform to world coordinates
                double wx = m_robot_x + lx * cos(m_robot_yaw) - ly * sin(m_robot_yaw);
                double wy = m_robot_y + lx * sin(m_robot_yaw) + ly * cos(m_robot_yaw);
                points.emplace_back(wx, wy);
            }
            angle += msg->angle_increment;
        }
        qDebug() << "Scan received:" << points.size() << "points";
        m_logText->append(QString("Scan received: %1 points").arg(points.size()));
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
        m_view->updateScan(points);
    }

    void updateObstacles(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        std::vector<QPointF> points;
        for (const auto &p : cloud.points) {
            // Assume relative to robot, transform to world
            double wx = m_robot_x + p.x * cos(m_robot_yaw) - p.y * sin(m_robot_yaw);
            double wy = m_robot_y + p.x * sin(m_robot_yaw) + p.y * cos(m_robot_yaw);
            points.emplace_back(wx, wy);
        }
        qDebug() << "Obstacles received:" << points.size() << "points";
        m_logText->append(QString("Obstacles received: %1 points").arg(points.size()));
        m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
        m_view->updateObstacles(points);
    }

    void toggleSpeedSubscription()
    {
        if (m_speedSubscribed) {
            m_speed_sub.reset();
            m_speedSubscribeBtn->setText("订阅");
            m_speedSubscribed = false;
            m_logText->append("Speed topic unsubscribed");
            m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
        } else {
            QString topic = m_speedTopicEdit->text();
            auto qos = rclcpp::QoS(10).best_effort(); // Use best_effort for cmd_vel
            m_speed_sub = m_node->create_subscription<geometry_msgs::msg::Twist>(
                topic.toStdString(), qos,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    updateSpeed(msg);
                });
            m_speedSubscribeBtn->setText("取消订阅");
            m_speedSubscribed = true;
            m_startTime = rclcpp::Clock().now().seconds();
            m_logText->append(QString("Speed topic subscribed: %1").arg(topic));
            m_logText->verticalScrollBar()->setValue(m_logText->verticalScrollBar()->maximum());
        }
    }

    void updateSpeed(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double currentTime = rclcpp::Clock().now().seconds() - m_startTime;
        double linear_x = msg->linear.x;
        double linear_y = msg->linear.y;
        double angular_z = msg->angular.z;
        emit speedUpdated(currentTime, linear_x, linear_y, angular_z);
    }

private:
    ImageView *m_view;
    QAction *m_moveAct;
    QSlider *m_brushSlider;
    QStatusBar *m_status;
    QString m_currentPath;
    QDockWidget *m_dock;
    QTableWidget *m_topicTable;
    std::vector<TopicInfo> m_topics;
    QCheckBox *m_showPathCheck;
    QCheckBox *m_showPointsCheck;
    QCheckBox *m_showArrowsCheck;
    QSpinBox *m_pointDensitySpin;
    QSpinBox *m_arrowDensitySpin;
    QTextEdit *m_logText;
    QLineEdit *m_speedTopicEdit;
    QPushButton *m_speedSubscribeBtn;
    SpeedPlot *m_speedPlot;
    bool m_speedSubscribed = false;
    std::shared_ptr<rclcpp::Node> m_node;
    rclcpp::executors::SingleThreadedExecutor m_executor;
    std::thread m_executor_thread;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_obstacles_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_speed_sub;
    double m_robot_x = 0.0;
    double m_robot_y = 0.0;
    double m_robot_yaw = 0.0;
    double m_startTime = 0.0;
    bool m_mapLoaded = false;
};

int main(int argc, char **argv)
{
    // ROS2 init before QApplication
    rclcpp::init(argc, argv);
    
    QApplication a(argc, argv);
    qRegisterMetaType<QTextCursor>();
    MainWindow w;
    w.show();
    int ret = a.exec();
    rclcpp::shutdown();
    return ret;
}

#include "main.moc"
