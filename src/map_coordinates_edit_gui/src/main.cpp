#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QFileDialog>
#include <QComboBox>
#include <QLineEdit>
#include <QMap>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QPainter>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QStyle>
#include <QStyleOptionGraphicsItem>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QScrollBar>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsSceneHoverEvent>
#include <QPixmap>
#include <QImage>
#include <QPointF>
#include <QPolygonF>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QMessageBox>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QEvent>
#include <QShowEvent>
#include <QResizeEvent>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrl>
#include <QUrlQuery>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QRegularExpression>
#include <QRandomGenerator>
#include <QDateTime>
#include <QSplitter>
#include <QToolBox>
#include <QScrollArea>
#include <QFrame>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QFormLayout>
#include <QSignalBlocker>
#include <climits>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

#include "map_coordinates_edit_gui/qnode.hpp"
#include "map_coordinates_edit_gui/keepout_db.hpp"
#include "map_coordinates_edit_gui/pgm_io.hpp"
#include "map_coordinates_edit_gui/pgm_assist.hpp"

// Tile map loader class
class TileMapLoader : public QObject {
    Q_OBJECT

public:
    TileMapLoader(QObject* parent = nullptr) : QObject(parent), manager_(new QNetworkAccessManager(this)) {
        connect(manager_, &QNetworkAccessManager::finished, this, &TileMapLoader::onTileDownloaded);
        // initialize counters and cache directory
        pendingTiles_ = 0;
        // Default local cache directory (absolute path providaed by user)
        cacheDir_ = QStringLiteral("/home/cdf/gps_filter_ws/src/tiledata");
        QDir(cacheDir_).mkpath(".");
        // Modest concurrency; TianDiTu browser keys still need rate limiting
        maxConcurrentRequests_ = 4;
    }

    // radius: number of tiles to load in each direction from center (e.g. 2 -> 5x5)
    void loadTileMap(const QString& baseUrl, int zoom, double centerLat, double centerLon, int tileSize = 256, int radius = 2) {
        baseUrl_ = baseUrl;
        zoom_ = zoom;
        requestedZoom_ = zoom_;
        centerLat_ = centerLat;
        centerLon_ = centerLon;
        tileSize_ = tileSize;

        // Drop any in-flight / queued work from a previous load so cache-only
        // views are not blocked by stale network retries.
        cancelPendingRequests();

        // Calculate tile coordinates for center
        int tileX = lon2tilex(centerLon_, zoom_);
        int tileY = lat2tiley(centerLat_, zoom_);

        // Load a square grid of tiles around center defined by radius
        // Clear only the cache for this requested zoom so we can keep lower-zoom
        // fallback tiles cached.
        tilesByZoom_[zoom_].clear();

        int cacheHits = 0;
        int parentHits = 0;
        int networkQueued = 0;

        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                int x = tileX + dx;
                int y = tileY + dy;
                // Work on a local copy so we don't mutate baseUrl_
                QString s = baseUrl_;
                s.replace("{level}", QString::number(zoom_));
                s.replace("{x}", QString::number(x));
                s.replace("{y}", QString::number(y));

                // Trim and remove surrounding quotes if present
                s = s.trimmed();
                if (s.startsWith('"') && s.endsWith('"')) {
                    s = s.mid(1, s.length() - 2).trimmed();
                }

                // For TianDiTu use a random subdomain t0..t7 if the URL contains t0
                if (s.contains("t0.")) {
                    QString subdomain = QString("t%1").arg(QRandomGenerator::global()->bounded(8));
                    s.replace("t0.", subdomain + ".");
                }

                // Normalize URL: remove accidental leading/trailing whitespace
                s = s.trimmed();
                // If scheme missing, try to add https
                QUrl q(s);
                if (!q.isValid() || q.scheme().isEmpty()) {
                    s = QString("https://") + s;
                    q = QUrl(s);
                }

                // Check local cache first: cacheDir_/z/x/y.png
                QString cachePath = QString("%1/%2/%3/%4.png").arg(cacheDir_).arg(zoom_).arg(x).arg(y);
                QFile cacheFile(cachePath);
                if (cacheFile.exists()) {
                    QPixmap pix;
                    if (pix.load(cachePath)) {
                        // If cache contains an error tile (server "no imagery" image), delete it and fallthrough to request
                        if (isLikelyErrorTile(pix)) {
                            qDebug() << "Cached tile looks like server error image, removing:" << cachePath;
                            cacheFile.remove();
                        } else {
                            tilesByZoom_[zoom_].insert(qMakePair(x, y), pix);
                            ++cacheHits;
                            continue; // exact cache hit: never queue network
                        }
                    }
                }

                // Attempt to use a parent tile from disk cache (avoid network request)
                bool parentUsed = false;
                for (int pz = zoom_ - 1; pz >= 1 && !parentUsed; --pz) {
                    int factor = 1 << (zoom_ - pz);
                    int parentX = x >> (zoom_ - pz);
                    int parentY = y >> (zoom_ - pz);
                    QString parentPath = QString("%1/%2/%3/%4.png").arg(cacheDir_).arg(pz).arg(parentX).arg(parentY);
                    QFile pfile(parentPath);
                    if (!pfile.exists()) continue;
                    QPixmap parentPix;
                    if (!parentPix.load(parentPath)) continue;
                    // if parent itself is an error image, skip it
                    if (isLikelyErrorTile(parentPix)) {
                        qDebug() << "Parent cache looks like server error image, skipping:" << parentPath;
                        continue;
                    }
                    QPixmap scaled = parentPix.scaled(tileSize_ * factor, tileSize_ * factor);
                    int cx = x % factor; if (cx < 0) cx += factor;
                    int cy = y % factor; if (cy < 0) cy += factor;
                    QRect srcRect(cx * tileSize_, cy * tileSize_, tileSize_, tileSize_);
                    // Ensure srcRect is within bounds of scaled pixmap
                    QRect scaledRect(0, 0, scaled.width(), scaled.height());
                    if (!scaledRect.contains(srcRect)) {
                        qDebug() << "Parent scaled image too small for child extraction, skipping parent:" << parentPath << "scaled size:" << scaled.size() << "needed rect:" << srcRect;
                        continue;
                    }
                    QPixmap child = scaled.copy(srcRect);
                    // Validate child tile: non-null, expected size, and not an error/placeholder
                    if (child.isNull() || child.width() != tileSize_ || child.height() != tileSize_ || isLikelyErrorTile(child)) {
                        qDebug() << "Parent-produced child tile invalid or looks like error, skipping parent:" << parentPath << "childSize:" << child.size();
                        continue;
                    }
                    tilesByZoom_[zoom_].insert(qMakePair(x, y), child);
                    qDebug() << "Loaded tile from parent cache:" << parentPath << "for" << cachePath;
                    parentUsed = true;
                    ++parentHits;
                    // Parent synthesis is treated as a cache hit: do NOT queue a
                    // background network refresh (that was causing slow 403 retries).
                    break;
                }
                if (parentUsed) continue;

                qDebug() << "Queueing URL:" << q.toString();
                PendingInfo info; info.x = x; info.y = y; info.z = zoom_; info.attempts = 0; info.backoffMs = 500;
                requestQueue_.append(qMakePair(s, info));
                ++networkQueued;
            }
        }

        qDebug() << "loadTileMap z=" << zoom_ << "radius=" << radius
                 << "cacheHits=" << cacheHits << "parentHits=" << parentHits
                 << "networkQueued=" << networkQueued;

        // Dispatch initial set of requests up to concurrency limit
        dispatchNextRequests();
        // If we had no network requests queued and no pending downloads,
        // we're done: notify listeners so the UI can update from cached tiles.
        if (pendingTiles_ == 0 && requestQueue_.isEmpty()) {
            QTimer::singleShot(0, this, [this]() {
                emit mapLoaded();
            });
        }
    }

    QPixmap getMapPixmap() const {
        if (!tilesByZoom_.contains(requestedZoom_) || tilesByZoom_[requestedZoom_].isEmpty()) return QPixmap();

        const int z = requestedZoom_;
        const auto &tiles = tilesByZoom_[z];

        // Determine bounds for requested zoom
        int minX = INT_MAX, minY = INT_MAX, maxX = INT_MIN, maxY = INT_MIN;
        for (auto it = tiles.constBegin(); it != tiles.constEnd(); ++it) {
            minX = std::min(minX, it.key().first);
            minY = std::min(minY, it.key().second);
            maxX = std::max(maxX, it.key().first);
            maxY = std::max(maxY, it.key().second);
        }

        int width = (maxX - minX + 1) * tileSize_;
        int height = (maxY - minY + 1) * tileSize_;

        QPixmap pixmap(width, height);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);

        // For each expected tile position in the bounding box, draw tile or try fallback
        for (int tx = minX; tx <= maxX; ++tx) {
            for (int ty = minY; ty <= maxY; ++ty) {
                QPoint key(tx, ty);
                int drawX = (tx - minX) * tileSize_;
                int drawY = (ty - minY) * tileSize_;

                if (tiles.contains(qMakePair(tx, ty))) {
                    painter.drawPixmap(drawX, drawY, tiles[qMakePair(tx, ty)]);
                    continue;
                }

                // Try to find a parent tile at lower zooms and extract the child region
                bool drawn = false;
                for (int pz = z - 1; pz >= 1 && !drawn; --pz) {
                    if (!tilesByZoom_.contains(pz)) continue;
                    int factor = 1 << (z - pz); // how many child tiles per parent tile
                    int parentX = tx >> (z - pz);
                    int parentY = ty >> (z - pz);
                    auto &parentTiles = tilesByZoom_[pz];
                    QPair<int,int> pkey = qMakePair(parentX, parentY);
                    if (!parentTiles.contains(pkey)) continue;

                    QPixmap parentPixmap = parentTiles[pkey];
                    if (parentPixmap.isNull()) continue;

                    // Scale parent to cover factor*tileSize in pixels
                    QPixmap scaled = parentPixmap.scaled(tileSize_ * factor, tileSize_ * factor);
                    // Child index within parent
                    int cx = tx % factor;
                    int cy = ty % factor;
                    if (cx < 0) cx += factor; if (cy < 0) cy += factor;
                    QRect srcRect(cx * tileSize_, cy * tileSize_, tileSize_, tileSize_);
                    QPixmap child = scaled.copy(srcRect);
                    painter.drawPixmap(drawX, drawY, child);
                    drawn = true;
                }

                // If nothing found, leave transparent (caller may display message)
            }
        }

        return pixmap;
    }

    QMap<QPair<int, int>, QPixmap> getTiles() const { 
        if (tilesByZoom_.contains(requestedZoom_)) return tilesByZoom_[requestedZoom_];
        return QMap<QPair<int, int>, QPixmap>();
    }

    // Heuristic to detect server "no imagery" tiles so we don't cache them.
    bool isLikelyErrorTile(const QPixmap &pix) const {
        if (pix.isNull()) return true;
        QImage img = pix.toImage().convertToFormat(QImage::Format_ARGB32);
        int w = img.width();
        int h = img.height();
        // quick reject: tiny images
        if (w < 16 || h < 16) return true;
        QSet<QRgb> colors;
        // sample pixels (not all) to keep it fast
        int stepX = std::max(1, w / 32);
        int stepY = std::max(1, h / 32);
        for (int y = 0; y < h; y += stepY) {
            const QRgb *line = reinterpret_cast<const QRgb*>(img.scanLine(y));
            for (int x = 0; x < w; x += stepX) {
                colors.insert(line[x]);
                if (colors.size() > 128) return false; // many colors -> likely real imagery
            }
        }
        // few unique sampled colors -> likely an error/watermark image
        return true;
    }

signals:
    void mapLoaded();

private slots:
    void onTileDownloaded(QNetworkReply* reply) {
        if (!pendingReplies_.contains(reply)) {
            reply->deleteLater();
            return;
        }
        PendingInfo info = pendingReplies_.take(reply);

        // Check HTTP status code if available
        QVariant httpStatus = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute);
        int statusCode = httpStatus.isValid() ? httpStatus.toInt() : 0;

        // Browser-key TianDiTu rejects requests without Referer/UA as 403.
        if (statusCode == 403) {
            qWarning() << "Received 403 Forbidden for tile" << info.x << info.y << "z" << info.z
                       << "- check tk type and request headers (User-Agent/Referer)";
            pendingTiles_--;
            if (pendingTiles_ <= 0) {
                pendingTiles_ = 0;
                emit mapLoaded();
            }
            reply->deleteLater();
            dispatchNextRequests();
            return;
        }

        if (statusCode == 429) {
            qDebug() << "Received 429 Too Many Requests for tile" << info.x << info.y << "z" << info.z;
            // schedule retry with exponential backoff if attempts remain
            // Increment global 429 counter and possibly enter a cooldown
            consecutive429Count_++;
            const int SUSPEND_THRESHOLD = 5;
            if (consecutive429Count_ >= SUSPEND_THRESHOLD && !suspendedRequests_) {
                suspendedRequests_ = true;
                qDebug() << "Too many 429s; suspending network requests for" << suspendCooldownMs_ << "ms";
                // schedule resume
                QTimer::singleShot(suspendCooldownMs_, this, [this]() {
                    suspendedRequests_ = false;
                    consecutive429Count_ = 0;
                    qDebug() << "Resuming network requests after cooldown";
                    dispatchNextRequests();
                });
            }
            // schedule per-tile retry with exponential backoff if attempts remain
            if (info.attempts < maxRetryAttempts_) {
                info.attempts++;
                // compute new backoff
                info.backoffMs = info.backoffMs <= 0 ? 500 : info.backoffMs * 2;
                QString s = baseUrl_;
                s.replace("{level}", QString::number(info.z));
                s.replace("{x}", QString::number(info.x));
                s.replace("{y}", QString::number(info.y));
                // trim and remove surrounding quotes if present
                s = s.trimmed();
                if (s.startsWith('"') && s.endsWith('"')) s = s.mid(1, s.length() - 2).trimmed();
                // If global suspension is active, just requeue without dispatching.
                QTimer::singleShot(info.backoffMs, this, [this, s, info]() mutable {
                    requestQueue_.append(qMakePair(s, info));
                    if (!suspendedRequests_) dispatchNextRequests();
                });
            } else {
                qDebug() << "Max retry attempts reached for tile" << info.x << info.y << "z" << info.z;
            }
            // consume reply
            pendingTiles_--;
            if (pendingTiles_ <= 0) {
                pendingTiles_ = 0;
                emit mapLoaded();
            }
            reply->deleteLater();
            // try dispatching queued requests (others may proceed)
            dispatchNextRequests();
            return;
        }

            if (reply->error() == QNetworkReply::NoError) {
            QByteArray data = reply->readAll();
            QPixmap tile;
            if (tile.loadFromData(data)) {
                // store tile under its zoom
                    // If the server returned an image that looks like an error/placeholder,
                    // do not cache it to disk and do not insert as valid tile.
                    if (isLikelyErrorTile(tile)) {
                        qDebug() << "Downloaded tile looks like server error image, skipping cache and attempting fallback:" << info.x << info.y << "z" << info.z;
                        // attempt to request a parent tile instead
                        if (info.z > 1) {
                            int parentZ = info.z - 1;
                            int parentX = info.x >> 1;
                            int parentY = info.y >> 1;
                            QString s = baseUrl_;
                            s.replace("{level}", QString::number(parentZ));
                            s.replace("{x}", QString::number(parentX));
                            s.replace("{y}", QString::number(parentY));
                            s = s.trimmed();
                            PendingInfo p; p.x = parentX; p.y = parentY; p.z = parentZ; p.attempts = 0; p.backoffMs = 500;
                            requestQueue_.append(qMakePair(s, p));
                        }
                    } else {
                        tilesByZoom_[info.z].insert(qMakePair(info.x, info.y), tile);
                        // Save to local cache so subsequent loads don't re-request from server
                        QString dirPath = QString("%1/%2/%3").arg(cacheDir_).arg(info.z).arg(info.x);
                        QDir d;
                        if (!d.mkpath(dirPath)) {
                            qDebug() << "Failed to create cache directory:" << dirPath;
                        }
                        QString cachePath = QString("%1/%2/%3/%4.png").arg(cacheDir_).arg(info.z).arg(info.x).arg(info.y);
                        if (!tile.save(cachePath, "PNG")) {
                            qDebug() << "Failed to save tile to cache:" << cachePath;
                        }
                        // Notify UI that a new tile is available so synthesized parent tiles
                        // can be replaced with the downloaded true tile promptly.
                        emit mapLoaded();
                    }
            } else {
                qDebug() << "Failed to load tile image data for" << info.x << info.y << "z" << info.z;
            }
        } else {
            qDebug() << "Tile download error for" << info.x << info.y << "z" << info.z << ":" << reply->errorString();
            // If tile missing and we can fallback, request parent tile at lower zoom
            if (info.z > 1) {
                int parentZ = info.z - 1;
                int parentX = info.x >> 1;
                int parentY = info.y >> 1;
                QString s = baseUrl_;
                s.replace("{level}", QString::number(parentZ));
                s.replace("{x}", QString::number(parentX));
                s.replace("{y}", QString::number(parentY));
                s = s.trimmed();
                PendingInfo p; p.x = parentX; p.y = parentY; p.z = parentZ; p.attempts = 0; p.backoffMs = 500;
                requestQueue_.append(qMakePair(s, p));
            }
        }

        pendingTiles_--;
        if (pendingTiles_ <= 0) {
            pendingTiles_ = 0;
            emit mapLoaded();
        }
        // try dispatching more queued requests
        dispatchNextRequests();

        reply->deleteLater();
    }

private:
    int lon2tilex(double lon, int z) {
        return (int)(floor((lon + 180.0) / 360.0 * (1 << z)));
    }

    int lat2tiley(double lat, int z) {
        double latrad = lat * M_PI / 180.0;
        return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z)));
    }

    double tilex2lon(int x, int z) {
        return (x / (double)(1 << z)) * 360.0 - 180.0;
    }

    double tiley2lat(int y, int z) {
        double n = M_PI - (y / (double)(1 << z)) * 2 * M_PI;
        return atan(sinh(n)) * 180.0 / M_PI;
    }

    QNetworkAccessManager* manager_;
    QString baseUrl_;
    int zoom_;
    double centerLat_, centerLon_;
    int tileSize_;
    // tiles organized by zoom level: tilesByZoom_[z][{x,y}] = pixmap
    QMap<int, QMap<QPair<int, int>, QPixmap>> tilesByZoom_;
    int pendingTiles_;
    QString cacheDir_;
    struct PendingInfo { int x; int y; int z; int attempts; int backoffMs; };
    QHash<QNetworkReply*, PendingInfo> pendingReplies_;
    int requestedZoom_ = 0;
    // Queue and concurrency control to avoid too many parallel requests to tile servers
    QList<QPair<QString, PendingInfo>> requestQueue_;
    int maxConcurrentRequests_ = 4;
    int maxRetryAttempts_ = 5;
    bool firstTileLoad_ = true;
    // Rate limiting: minimum interval between issuing requests (ms)
    int minRequestIntervalMs_ = 80;
    qint64 lastRequestTimestampMs_ = 0;
    // Rate-limit defense: when many 429s are observed, suspend all
    // network dispatch for a cooldown period to avoid hammering the server.
    int consecutive429Count_ = 0;
    bool suspendedRequests_ = false;
    int suspendCooldownMs_ = 60000; // 60s cooldown by default

    // TianDiTu "浏览器端" keys require browser-like headers.
    void applyTileRequestHeaders(QNetworkRequest& request) const {
        request.setRawHeader(
            "User-Agent",
            "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) "
            "Chrome/120.0.0.0 Safari/537.36");
        request.setRawHeader("Referer", "https://www.tianditu.gov.cn/");
        request.setRawHeader("Accept", "image/avif,image/webp,image/apng,image/*,*/*;q=0.8");
    }

    void cancelPendingRequests() {
        requestQueue_.clear();
        const QList<QNetworkReply*> replies = pendingReplies_.keys();
        pendingReplies_.clear();
        pendingTiles_ = 0;
        for (QNetworkReply* reply : replies) {
            if (!reply) continue;
            reply->abort();
            reply->deleteLater();
        }
    }

    void dispatchNextRequests() {
        // If we've suspended requests due to repeated 429s, don't dispatch now.
        if (suspendedRequests_) {
            qDebug() << "dispatchNextRequests: suspended due to rate limit cooldown";
            return;
        }

        // Issue requests with a minimum interval to avoid bursting and hitting server rate limits.
        while (!requestQueue_.isEmpty() && pendingTiles_ < maxConcurrentRequests_) {
            auto item = requestQueue_.takeFirst();
            QString s = item.first;
            PendingInfo info = item.second;

            qint64 now = QDateTime::currentMSecsSinceEpoch();
            qint64 elapsed = now - lastRequestTimestampMs_;
            int delay = 0;
            if (elapsed < minRequestIntervalMs_) delay = static_cast<int>(minRequestIntervalMs_ - elapsed);

            // Reserve a pending slot now so other dispatchers don't oversubscribe
            pendingTiles_++;

            QTimer::singleShot(delay, this, [this, s, info]() mutable {
                QString ss = s;
                QUrl q(ss);
                if (!q.isValid() || q.scheme().isEmpty()) {
                    ss = QString("https://") + ss;
                    q = QUrl(ss);
                }
                QNetworkRequest request(q);
                applyTileRequestHeaders(request);
                QNetworkReply* reply = manager_->get(request);
                pendingReplies_[reply] = info;
                lastRequestTimestampMs_ = QDateTime::currentMSecsSinceEpoch();
            });
        }
    }
};

// Custom graphics item for drawing shapes
class ShapeItem : public QGraphicsItem {
public:
    enum ShapeType { Line, Rectangle, Circle, Polygon };

    /** Nav2 keepout_filter inflation preview (meters → scene via metersPerPixel). */
    static void setInflationPreview(bool show, double inscribed_m, double inflation_m, double meters_per_pixel) {
        s_showInflation_ = show;
        s_inscribedM_ = std::max(0.0, inscribed_m);
        s_inflationM_ = std::max(0.0, inflation_m);
        s_metersPerPixel_ = std::max(1e-9, meters_per_pixel);
    }
    static bool inflationPreviewEnabled() { return s_showInflation_; }

    void notifyInflationPreviewChanged() {
        prepareGeometryChange();
        update();
    }

    ShapeItem(ShapeType type, const QPolygonF& points = QPolygonF())
        : type_(type), points_(points), finished_(false), radius_(0.0), mode_(IM_None) {
        setFlag(QGraphicsItem::ItemIsSelectable, true);
        setFlag(QGraphicsItem::ItemIsMovable, false);
        setAcceptHoverEvents(true);
        setZValue(10);  // above tile/map pixmap
        switch (type_) {
            case Line: dbType_ = "line"; break;
            case Rectangle: dbType_ = "rectangle"; break;
            case Circle: dbType_ = "circle"; break;
            case Polygon: dbType_ = "polygon"; break;
        }
    }

    /** Pinned = locked in place (cannot move / resize / rotate). */
    void setPinned(bool pinned) {
        pinned_ = pinned;
        applyMovableFlag();
        update();
    }
    bool isPinned() const { return pinned_; }

    void applyMovableFlag() {
        setFlag(QGraphicsItem::ItemIsMovable, finished_ && !pinned_);
    }

    void addPoint(const QPointF& point) {
        points_.append(point);
        if (type_ == Rectangle && points_.size() >= 2) {
            // Two corners define the rect (also used when loading finished shapes).
            prepareGeometryChange();
            finished_ = true;
            center_ = (points_.first() + points_.last()) / 2.0;
            setTransformOriginPoint(center_);
            pinned_ = true;  // lock after draw; unpin to move
            applyMovableFlag();
        } else if (type_ == Line && points_.size() >= 2) {
            // Straight segment: exactly two endpoints.
            prepareGeometryChange();
            finished_ = true;
            pinned_ = true;
            applyMovableFlag();
        } else if (type_ == Circle) {
            center_ = points_.first();
            if (points_.size() >= 2) {
                radius_ = QLineF(center_, points_.last()).length();
                finished_ = true;
                pinned_ = true;
                applyMovableFlag();
            }
        } else if (type_ == Polygon) {
            // polygon finishes only by explicit right-click / Enter
        }
        update();
    }

    /** Live preview endpoint for Line / Rectangle while dragging. */
    void setEndpoint(const QPointF & point) {
        prepareGeometryChange();
        if (points_.isEmpty()) {
            points_.append(point);
        } else if (points_.size() == 1) {
            points_.append(point);
        } else {
            points_[points_.size() - 1] = point;
        }
        if (type_ == Rectangle && points_.size() >= 2) {
            center_ = (points_.first() + points_.last()) / 2.0;
        }
        update();
    }

    void setRadius(double r) { radius_ = r; update(); }
    QPointF getCenter() const { return center_; }
    int pointCount() const { return points_.size(); }

    /** Remove the last vertex / cancel finish; returns false if nothing to undo. */
    bool undoLastVertex() {
        if (points_.isEmpty() && !finished_) {
            return false;
        }
        prepareGeometryChange();
        if (finished_ && (type_ == Rectangle || type_ == Circle)) {
            // Undo completing the shape: drop back to in-progress with first point only
            finished_ = false;
            pinned_ = false;
            applyMovableFlag();
            setRotation(0);
            setTransform(QTransform());
            if (type_ == Circle) {
                radius_ = 0.0;
                if (points_.size() > 1) {
                    points_ = QPolygonF() << points_.first();
                }
                if (!points_.isEmpty()) {
                    center_ = points_.first();
                }
            } else if (type_ == Rectangle) {
                if (points_.size() > 1) {
                    points_ = QPolygonF() << points_.first();
                }
            }
            update();
            return true;
        }
        if (points_.isEmpty()) {
            return false;
        }
        points_.removeLast();
        finished_ = false;
        pinned_ = false;
        applyMovableFlag();
        if (type_ == Circle) {
            radius_ = 0.0;
            if (!points_.isEmpty()) {
                center_ = points_.first();
            }
        }
        update();
        return true;
    }

    void finish() {
        prepareGeometryChange();
        finished_ = true;
        if (type_ == Rectangle && points_.size() >= 2) {
            center_ = (points_.first() + points_.last()) / 2.0;
            setTransformOriginPoint(center_);
        } else if (type_ == Circle && points_.size() >= 1) {
            center_ = points_.first();
        }
        pinned_ = true;  // lock after finish; user must unpin to move
        applyMovableFlag();
        update();
    }

    /** Finalize if geometry is already valid for export (used by Save/Publish). */
    bool finishIfReady() {
        if (finished_) {
            return true;
        }
        if (type_ == Line && points_.size() >= 2) {
            finish();
            return true;
        }
        if (type_ == Polygon && points_.size() >= 3) {
            finish();
            return true;
        }
        if (type_ == Rectangle && points_.size() >= 2) {
            finish();
            return true;
        }
        if (type_ == Circle && (radius_ > 0.0 || points_.size() >= 2)) {
            if (points_.size() >= 1) {
                center_ = points_.first();
            }
            finish();
            return true;
        }
        return false;
    }

    bool isFinished() const { return finished_; }

    ShapeType shapeType() const { return type_; }

    /** Stable DB figure_type string set at construction (line/rectangle/circle/polygon). */
    const char * dbFigureType() const { return dbType_; }

    double radiusLocal() const { return radius_; }

    /** Vertices in scene coordinates (works for in-progress shapes too). */
    QVector<QPointF> allScenePoints() const {
        QVector<QPointF> out;
        for (const auto & p : points_) {
            out.push_back(mapToScene(p));
        }
        return out;
    }

    /** Finished geometry vertices in scene coordinates (empty for Circle). */
    QVector<QPointF> sceneVertices() const {
        QVector<QPointF> out;
        if (!finished_) {
            return out;
        }
        if (type_ == Circle) {
            return out;
        }
        if (type_ == Rectangle && points_.size() >= 2) {
            QRectF rect(points_.first(), points_.last());
            rect = rect.normalized();
            const QPointF corners[4] = {
                rect.topLeft(), rect.topRight(), rect.bottomRight(), rect.bottomLeft()};
            for (const auto & c : corners) {
                out.push_back(mapToScene(c));
            }
            return out;
        }
        return allScenePoints();
    }

    QPointF sceneCenter() const {
        if (type_ == Circle && !points_.isEmpty()) {
            return mapToScene(points_.first());
        }
        return mapToScene(center_);
    }

    /** Circle radius mapped to scene units. */
    double sceneRadius() const {
        if (radius_ <= 0.0) {
            return 0.0;
        }
        QPointF rim = center_ + QPointF(radius_, 0.0);
        return QLineF(mapToScene(center_), mapToScene(rim)).length();
    }

    /**
     * Replace geometry with new scene-space coordinates (item at origin).
     * @param markFinished if false, keep the shape in-progress (for mid-draw remap).
     */
    void applySceneGeometry(const QVector<QPointF> & scenePts,
                            const QPointF & sceneCenter, double sceneRadius,
                            bool markFinished = true) {
        prepareGeometryChange();
        setPos(0, 0);
        setRotation(0);
        setTransform(QTransform());
        points_.clear();
        finished_ = markFinished;
        setZValue(10);

        if (type_ == Circle) {
            center_ = sceneCenter;
            radius_ = sceneRadius;
            points_.append(sceneCenter);
            if (sceneRadius > 0.0 || markFinished) {
                points_.append(sceneCenter + QPointF(sceneRadius, 0.0));
            }
            if (markFinished && !pinned_) {
                // keep existing pin; newly remapped finished shapes stay pinned if already
            }
            applyMovableFlag();
            update();
            return;
        }

        if (type_ == Rectangle && scenePts.size() >= 4) {
            // Preserve orientation: 4 corners (TL,TR,BR,BL) from sceneVertices().
            applyOrientedRectFromSceneCorners(scenePts);
        } else if (type_ == Rectangle && scenePts.size() >= 2) {
            QRectF r(scenePts.first(), scenePts.first());
            for (const auto & p : scenePts) {
                r = r.united(QRectF(p, p));
            }
            points_.append(r.topLeft());
            points_.append(r.bottomRight());
            center_ = r.center();
            setTransformOriginPoint(center_);
        } else {
            for (const auto & p : scenePts) {
                points_.append(p);
            }
            if (!points_.isEmpty()) {
                center_ = points_.first();
            }
        }
        applyMovableFlag();
        update();
    }

    /** Rebuild an oriented rectangle from 4 scene-space corners (order around boundary). */
    void applyOrientedRectFromSceneCorners(const QVector<QPointF> & corners) {
        const QPointF tl = corners[0];
        const QPointF tr = corners[1];
        const QPointF br = corners[2];
        // bl = corners[3]
        const QPointF center = (tl + tr + br + corners[3]) * 0.25;
        const double angleRad = std::atan2(tr.y() - tl.y(), tr.x() - tl.x());
        const double w = QLineF(tl, tr).length();
        const double h = QLineF(tr, br).length();
        points_.clear();
        points_.append(QPointF(center.x() - w * 0.5, center.y() - h * 0.5));
        points_.append(QPointF(center.x() + w * 0.5, center.y() + h * 0.5));
        center_ = center;
        setTransformOriginPoint(center_);
        setRotation(angleRad * 180.0 / M_PI);
    }

    // Precise hit test in scene coordinates to decide whether a mouse press
    // should begin a move/resize/rotate operation on this item.
    bool acceptsMoveAtScenePos(const QPointF &scenePos) {
        QPointF local = mapFromScene(scenePos);
        return acceptsMoveAtLocalPoint(local);
    }

    // expose for unit-style checks
    bool acceptsMoveAtLocalPoint(const QPointF &local) const {
        // If user clicked a handle (corner/vertex/rotation), allow interaction
        if (hitTestHandles(local)) return true;

        // For different shape types require being on the shape (line) or
        // strictly inside the filled area (rectangle/polygon/circle).
        if (type_ == Line) {
            // Check distance to any segment
            const double THRESH = 6.0; // pixels in item-local coordinates
            if (points_.size() < 2) return false;
            for (int i = 0; i < points_.size() - 1; ++i) {
                const QPointF &a = points_.at(i);
                const QPointF &b = points_.at(i + 1);
                if (pointToSegmentDistance(local, a, b) <= THRESH) return true;
            }
            return false;
        } else if (type_ == Rectangle) {
            if (points_.size() < 2) return false;
            QRectF rect(points_.first(), points_.last()); rect = rect.normalized();
            // require strict interior (not just bounding rect)
            return rect.contains(local);
        } else if (type_ == Polygon) {
            if (points_.isEmpty()) return false;
            return QPolygonF(points_).containsPoint(local, Qt::WindingFill);
        } else if (type_ == Circle) {
            if (!finished_) return false;
            double d = QLineF(center_, local).length();
            return d <= radius_;
        }
        return false;
    }

private:
    // distance from point p to segment ab
    static double pointToSegmentDistance(const QPointF &p, const QPointF &a, const QPointF &b) {
        double vx = b.x() - a.x();
        double vy = b.y() - a.y();
        double wx = p.x() - a.x();
        double wy = p.y() - a.y();
        double c1 = vx * wx + vy * wy;
        if (c1 <= 0.0) return std::hypot(p.x() - a.x(), p.y() - a.y());
        double c2 = vx * vx + vy * vy;
        if (c2 <= c1) return std::hypot(p.x() - b.x(), p.y() - b.y());
        double t = c1 / c2;
        double projx = a.x() + t * vx;
        double projy = a.y() + t * vy;
        return std::hypot(p.x() - projx, p.y() - projy);
    }

    bool hitTestHandles(const QPointF &local) const {
        // For rectangle: corner handles and rotation handle near center
        const qreal H = 6.0;
        if (type_ == Rectangle && points_.size() >= 2) {
            QRectF rect(points_.first(), points_.last()); rect = rect.normalized();
            QRectF tl(rect.topLeft() - QPointF(H/2, H/2), QSizeF(H, H));
            QRectF tr(rect.topRight() - QPointF(H/2, H/2), QSizeF(H, H));
            QRectF bl(rect.bottomLeft() - QPointF(H/2, H/2), QSizeF(H, H));
            QRectF br(rect.bottomRight() - QPointF(H/2, H/2), QSizeF(H, H));
            if (tl.contains(local) || tr.contains(local) || bl.contains(local) || br.contains(local)) return true;
            // rotation handle near center (previously within 10 px)
            QPointF center((rect.left()+rect.right())/2.0, (rect.top()+rect.bottom())/2.0);
            if (QLineF(center, local).length() <= 10.0) return true;
        }
        // Polygon vertices
        if (type_ == Polygon && !points_.isEmpty()) {
            const double VTH = 6.0;
            for (const QPointF &pt : points_) if (QLineF(pt, local).length() <= VTH) return true;
        }
        // Circle: treat rotation/move control if click near center or edge handle
        if (type_ == Circle && finished_) {
            if (QLineF(center_, local).length() <= 10.0) return true;
        }
        return false;
    }

public:
    QRectF boundingRect() const override {
        QRectF r;
        if (type_ == Circle && (finished_ || radius_ > 0.0)) {
            const QPointF c = points_.isEmpty() ? center_ : points_.first();
            r = QRectF(c.x() - radius_, c.y() - radius_, 2 * radius_, 2 * radius_).adjusted(-8, -8, 8, 8);
        } else if (points_.isEmpty()) {
            return QRectF();
        } else {
            r = points_.boundingRect().adjusted(-8, -8, 8, 8);
            if (type_ == Rectangle && finished_) {
                r.adjust(-12, -28, 12, 12);
            }
        }
        if (s_showInflation_ && finished_ && s_metersPerPixel_ > 1e-9) {
            const double pad = s_inflationM_ / s_metersPerPixel_ + 4.0;
            r.adjust(-pad, -pad, pad, pad);
        }
        return r;
    }

    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget*) override {
        // Main shape pen: unfinished = dashed (needs right-click / Save to finalize).
        // Line uses a thinner cosmetic stroke; filled shapes keep a slightly wider outline.
        const bool isLine = (type_ == Line);
        const qreal baseWidth = isLine ? 1.0 : 2.0;
        const qreal selectedWidth = isLine ? 1.5 : 2.5;
        QPen shapePen(Qt::red, baseWidth);
        shapePen.setCosmetic(true);  // constant screen pixels under zoom
        if (!finished_) {
            shapePen.setStyle(Qt::DashLine);
        }
        if (option && (option->state & QStyle::State_Selected)) {
            shapePen.setColor(QColor(30, 120, 255));
            shapePen.setWidthF(selectedWidth);
        }
        painter->setPen(shapePen);
        // Lines: no fill brush (avoids looking like thick bands)
        if (isLine) {
            painter->setBrush(Qt::NoBrush);
        } else {
            const int fillAlpha = finished_ ? 50 : 25;
            QColor fill = (option && (option->state & QStyle::State_Selected))
                ? QColor(30, 120, 255, fillAlpha)
                : QColor(255, 0, 0, fillAlpha);
            painter->setBrush(QBrush(fill));
        }

        // Nav2 inflation bands under the keepout geometry
        if (s_showInflation_ && finished_ && s_metersPerPixel_ > 1e-9) {
            paintInflationPreview(painter);
            // restore pens after preview
            painter->setPen(shapePen);
            if (isLine) {
                painter->setBrush(Qt::NoBrush);
            } else {
                const int fillAlpha = finished_ ? 50 : 25;
                QColor fill = (option && (option->state & QStyle::State_Selected))
                    ? QColor(30, 120, 255, fillAlpha)
                    : QColor(255, 0, 0, fillAlpha);
                painter->setBrush(QBrush(fill));
            }
        }

        // Capture item→device transform before any handle drawing that resets it.
        const QTransform wt = painter->worldTransform();

        if (type_ == Line && points_.size() >= 2) {
            for (int i = 0; i < points_.size() - 1; ++i) painter->drawLine(points_[i], points_[i+1]);
        } else if (type_ == Rectangle && points_.size() >= 2) {
            QRectF rect(points_.first(), points_.last()); rect = rect.normalized();
            painter->drawRect(rect);
            if (finished_ && !pinned_) {
                // Draw handles in device pixels so size stays constant under zoom+rotation.
                painter->save();
                painter->setWorldTransform(QTransform());
                QPen handlePen(Qt::black);
                handlePen.setWidth(1);
                painter->setPen(handlePen);
                painter->setBrush(QBrush(Qt::white));
                constexpr double hs = 6.0;  // half-size in screen pixels
                const QPointF corners[4] = {
                    rect.topLeft(), rect.topRight(), rect.bottomLeft(), rect.bottomRight()
                };
                for (const QPointF & c : corners) {
                    const QPointF d = wt.map(c);
                    painter->drawRect(QRectF(d.x() - hs, d.y() - hs, 2.0 * hs, 2.0 * hs));
                }
                const QPointF center((rect.left() + rect.right()) / 2.0,
                                     (rect.top() + rect.bottom()) / 2.0);
                const QPointF dc = wt.map(center);
                QPen rotPen(Qt::darkYellow);
                rotPen.setWidth(1);
                painter->setPen(rotPen);
                painter->setBrush(QBrush(Qt::yellow));
                painter->drawEllipse(dc, hs, hs);
                painter->restore();
            }
        } else if (type_ == Circle && (finished_ || radius_ > 0.0)) {
            const QPointF c = points_.isEmpty() ? center_ : points_.first();
            painter->drawEllipse(c, radius_, radius_);
        } else if (type_ == Polygon && points_.size() >= 2) {
            // Draw polyline while in progress; closed fill once >= 3 points.
            if (points_.size() >= 3) {
                painter->drawPolygon(points_);
            } else {
                painter->drawPolyline(points_);
            }
        }

        // Vertex markers (skip finished rectangle — corner handles already mark them).
        const bool skipVertexMarks = (type_ == Rectangle && finished_);
        if (!skipVertexMarks && !points_.isEmpty()) {
            painter->save();
            painter->setWorldTransform(QTransform());
            QPen vertPen(Qt::blue);
            vertPen.setWidth(1);
            painter->setPen(vertPen);
            painter->setBrush(QBrush(Qt::blue));
            constexpr double vr = 3.0;
            for (const QPointF & point : points_) {
                const QPointF d = wt.map(point);
                painter->drawEllipse(d, vr, vr);
            }
            painter->restore();
        }

        // Pin indicator (locked) — also device-pixel sized.
        if (finished_ && pinned_) {
            QPointF pinAt;
            if (type_ == Circle) {
                pinAt = points_.isEmpty() ? center_ : points_.first();
            } else if (type_ == Rectangle && points_.size() >= 2) {
                pinAt = QRectF(points_.first(), points_.last()).normalized().center();
            } else if (!points_.isEmpty()) {
                pinAt = points_.boundingRect().center();
            } else {
                pinAt = center_;
            }
            painter->save();
            painter->setWorldTransform(QTransform());
            const QPointF d = wt.map(pinAt);
            constexpr double r = 7.0;
            painter->setPen(QPen(QColor(40, 40, 40), 1.5));
            painter->setBrush(QBrush(QColor(220, 50, 50)));
            painter->drawEllipse(d, r, r);
            painter->setPen(QPen(Qt::white, 1.5));
            painter->drawLine(d + QPointF(0, -r * 0.4), d + QPointF(0, r * 0.55));
            painter->restore();
        }
    }

    // Interaction
    enum InteractionMode { IM_None, IM_Move, IM_Resize_TL, IM_Resize_TR, IM_Resize_BL, IM_Resize_BR, IM_Rotate };

    void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override {
        if (pinned_ || !finished_) {
            setCursor(Qt::ArrowCursor);
            QGraphicsItem::hoverMoveEvent(event);
            return;
        }
        if (type_ == Rectangle && finished_) {
            QRectF rect(points_.first(), points_.last()); rect = rect.normalized();
            const qreal H = 8.0;
            QRectF tl(rect.topLeft()-QPointF(H/2,H/2), QSizeF(H,H));
            QRectF tr(rect.topRight()-QPointF(H/2,H/2), QSizeF(H,H));
            QRectF bl(rect.bottomLeft()-QPointF(H/2,H/2), QSizeF(H,H));
            QRectF br(rect.bottomRight()-QPointF(H/2,H/2), QSizeF(H,H));
            QPointF pos = event->pos();
            if (tl.contains(pos) || br.contains(pos)) setCursor(Qt::SizeFDiagCursor);
            else if (tr.contains(pos) || bl.contains(pos)) setCursor(Qt::SizeBDiagCursor);
            else {
                QPointF center((rect.left()+rect.right())/2.0, (rect.top()+rect.bottom())/2.0);
                if (QLineF(center, pos).length() <= 10) setCursor(Qt::CrossCursor);
                else setCursor(Qt::OpenHandCursor);
            }
        }
        QGraphicsItem::hoverMoveEvent(event);
    }

    void mousePressEvent(QGraphicsSceneMouseEvent* event) override {
        if (!finished_) { QGraphicsItem::mousePressEvent(event); return; }
        setSelected(true);
        setZValue(20);
        if (pinned_) {
            mode_ = IM_None;
            QGraphicsItem::mousePressEvent(event);
            return;
        }
        if (type_ == Rectangle) {
            QRectF rect(points_.first(), points_.last()); rect = rect.normalized();
            const qreal H = 8.0;
            QRectF tl(rect.topLeft()-QPointF(H/2,H/2), QSizeF(H,H));
            QRectF tr(rect.topRight()-QPointF(H/2,H/2), QSizeF(H,H));
            QRectF bl(rect.bottomLeft()-QPointF(H/2,H/2), QSizeF(H,H));
            QRectF br(rect.bottomRight()-QPointF(H/2,H/2), QSizeF(H,H));
            QPointF pos = event->pos();
            if (tl.contains(pos)) { mode_ = IM_Resize_TL; resizeAnchor_ = rect.bottomRight(); event->accept(); return; }
            if (tr.contains(pos)) { mode_ = IM_Resize_TR; resizeAnchor_ = rect.bottomLeft(); event->accept(); return; }
            if (bl.contains(pos)) { mode_ = IM_Resize_BL; resizeAnchor_ = rect.topRight(); event->accept(); return; }
            if (br.contains(pos)) { mode_ = IM_Resize_BR; resizeAnchor_ = rect.topLeft(); event->accept(); return; }
            QPointF center((rect.left()+rect.right())/2.0, (rect.top()+rect.bottom())/2.0);
            if (QLineF(center, pos).length() <= 10) {
                mode_ = IM_Rotate;
                rotationAtPress_ = rotation();
                rotationAccum_ = 0.0;
                rotateLastMousePos_ = event->scenePos();
                center_ = center;
                setTransformOriginPoint(center_);  // must be LOCAL, not scene
                event->accept();
                return;
            }
            mode_ = IM_Move; QGraphicsItem::mousePressEvent(event); return;
        }
        QGraphicsItem::mousePressEvent(event);
    }

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override {
        if (!finished_ || pinned_) { QGraphicsItem::mouseMoveEvent(event); return; }
        if (type_ == Rectangle) {
            QPointF p = event->pos();
            if (mode_ == IM_Resize_TL) {
                prepareGeometryChange(); points_.first() = p;
                center_ = (points_.first() + points_.last()) / 2.0;
                setTransformOriginPoint(center_); update(); return;
            }
            if (mode_ == IM_Resize_TR) {
                prepareGeometryChange(); points_.first().setY(p.y()); points_.last().setX(p.x());
                center_ = (points_.first() + points_.last()) / 2.0;
                setTransformOriginPoint(center_); update(); return;
            }
            if (mode_ == IM_Resize_BL) {
                prepareGeometryChange(); points_.first().setX(p.x()); points_.last().setY(p.y());
                center_ = (points_.first() + points_.last()) / 2.0;
                setTransformOriginPoint(center_); update(); return;
            }
            if (mode_ == IM_Resize_BR) {
                prepareGeometryChange(); points_.last() = p;
                center_ = (points_.first() + points_.last()) / 2.0;
                setTransformOriginPoint(center_); update(); return;
            }
            if (mode_ == IM_Rotate) {
                QPointF centerScene = mapToScene(center_);
                QPointF lastScene = rotateLastMousePos_;
                QPointF curScene = event->scenePos();
                double a1 = std::atan2(lastScene.y() - centerScene.y(), lastScene.x() - centerScene.x());
                double a2 = std::atan2(curScene.y() - centerScene.y(), curScene.x() - centerScene.x());
                double deltaRad = a2 - a1;
                while (deltaRad > M_PI) deltaRad -= 2 * M_PI;
                while (deltaRad < -M_PI) deltaRad += 2 * M_PI;
                double deltaDeg = deltaRad * 180.0 / M_PI;
                const double MIN_DEG = 0.25;
                if (std::fabs(deltaDeg) >= MIN_DEG) {
                    rotationAccum_ += deltaDeg;
                    setTransformOriginPoint(center_);  // LOCAL
                    setRotation(rotationAtPress_ + rotationAccum_);
                    update();
                }
                rotateLastMousePos_ = curScene;
                return;
            }
            if (mode_ == IM_Move) { QGraphicsItem::mouseMoveEvent(event); return; }
        }
        QGraphicsItem::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override {
        mode_ = IM_None; QGraphicsItem::mouseReleaseEvent(event);
    }

    void paintInflationPreview(QPainter * painter) const {
        const double infPx = s_inflationM_ / s_metersPerPixel_;
        const double insPx = s_inscribedM_ / s_metersPerPixel_;
        if (infPx < 0.5 && insPx < 0.5) {
            return;
        }

        auto strokeBand = [&](const QPainterPath & outline, double halfWidthPx, const QColor & fill) {
            if (halfWidthPx < 0.5 || outline.isEmpty()) {
                return;
            }
            QPainterPathStroker stroker;
            stroker.setWidth(2.0 * halfWidthPx);
            stroker.setJoinStyle(Qt::RoundJoin);
            stroker.setCapStyle(Qt::RoundCap);
            painter->fillPath(stroker.createStroke(outline), fill);
        };

        painter->save();
        painter->setPen(Qt::NoPen);

        if (type_ == Circle) {
            const QPointF c = points_.isEmpty() ? center_ : points_.first();
            if (infPx > 0.5) {
                painter->setBrush(QColor(0, 200, 255, 45));
                painter->setPen(QPen(QColor(0, 180, 220, 200), 0));
                painter->drawEllipse(c, radius_ + infPx, radius_ + infPx);
            }
            if (insPx > 0.5) {
                painter->setBrush(QColor(255, 160, 0, 55));
                painter->setPen(QPen(QColor(230, 120, 0, 200), 0));
                painter->drawEllipse(c, radius_ + insPx, radius_ + insPx);
            }
            // Keep original keepout visible on top (caller already drew it before this)
        } else {
            QPainterPath outline;
            if (type_ == Line && points_.size() >= 2) {
                outline.moveTo(points_[0]);
                for (int i = 1; i < points_.size(); ++i) {
                    outline.lineTo(points_[i]);
                }
            } else if (type_ == Rectangle && points_.size() >= 2) {
                outline.addRect(QRectF(points_.first(), points_.last()).normalized());
            } else if (type_ == Polygon && points_.size() >= 2) {
                outline.addPolygon(points_);
                outline.closeSubpath();
            }
            // Outer inflation first, then inscribed (drawn on top)
            strokeBand(outline, infPx, QColor(0, 200, 255, 45));
            strokeBand(outline, insPx, QColor(255, 160, 0, 55));
        }
        painter->restore();
    }

private:
    ShapeType type_;
    const char * dbType_{"polygon"};
    QPolygonF points_;
    bool finished_;
    bool pinned_{true};
    QPointF center_;
    double radius_;
    // interaction state
    InteractionMode mode_;
    QPointF resizeAnchor_;
    qreal rotationStart_ = 0.0;
    // rotation interaction helpers
    QPointF rotateLastMousePos_;
    qreal rotationAtPress_ = 0.0;
    qreal rotationAccum_ = 0.0;
    qint64 rotateLastTimestampMs_ = 0;
    double rotateSpeedScale_ = 0.06;

    static bool s_showInflation_;
    static double s_inscribedM_;
    static double s_inflationM_;
    static double s_metersPerPixel_;
};

bool ShapeItem::s_showInflation_ = false;
double ShapeItem::s_inscribedM_ = 0.40;
double ShapeItem::s_inflationM_ = 1.0;
double ShapeItem::s_metersPerPixel_ = 1.0;

class MapGraphicsView : public QGraphicsView {
    Q_OBJECT

public:
    MapGraphicsView(QGraphicsScene* scene, QWidget* parent = nullptr)
        : QGraphicsView(scene, parent), currentShape_(nullptr), drawingMode_(false), pressed_(false), panning_(false) {
        setMouseTracking(true);
        setRenderHint(QPainter::Antialiasing);
        setInteractive(true);
        setDragMode(QGraphicsView::NoDrag);
        // Avoid AlignCenter clamping one axis (causes horizontal pan to drift vertically).
        setAlignment(Qt::AlignLeft | Qt::AlignTop);
        setTransformationAnchor(QGraphicsView::AnchorViewCenter);
        setResizeAnchor(QGraphicsView::AnchorViewCenter);
        // Hidden but still used for 1:1 grab-hand panning via setValue().
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setFocusPolicy(Qt::StrongFocus);
    }

    // Minimum visual scale under which new shape creation is disabled
    // (when zoomed out too far handles become impractical).
    double minVisualScaleForDrawing_ = 0.25;

    void setDrawingMode(bool enabled, ShapeItem::ShapeType type = ShapeItem::Line) {
        if (currentShape_ && enabled && type != currentShapeType_) {
            // Tool changed: discard in-progress shape.
            scene()->removeItem(currentShape_);
            delete currentShape_;
            currentShape_ = nullptr;
            pressed_ = false;
        }
        if (!enabled && currentShape_) {
            // Stop Drawing: finalize if valid, otherwise discard.
            if (currentShape_->finishIfReady()) {
                emit shapeFinished();
            } else {
                scene()->removeItem(currentShape_);
                delete currentShape_;
            }
            currentShape_ = nullptr;
            pressed_ = false;
        }
        drawingMode_ = enabled;
        currentShapeType_ = type;
        setCursor(enabled ? Qt::CrossCursor : Qt::ArrowCursor);
    }

    bool isDrawingMode() const { return drawingMode_; }

    void refreshInflationPreview(bool show, double inscribed_m, double inflation_m, double meters_per_pixel) {
        ShapeItem::setInflationPreview(show, inscribed_m, inflation_m, meters_per_pixel);
        for (ShapeItem * shape : getAllShapeItems()) {
            shape->notifyInflationPreviewChanged();
        }
    }

    ShapeItem * findShapeAt(const QPointF & scenePos) const {
        for (QGraphicsItem * item : scene()->items(scenePos)) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (shape && shape->isFinished() && shape->acceptsMoveAtScenePos(scenePos)) {
                return shape;
            }
        }
        // Looser fallback: any finished shape whose scene bounding rect contains the point
        for (QGraphicsItem * item : scene()->items(scenePos)) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (shape && shape->isFinished()) {
                return shape;
            }
        }
        return nullptr;
    }

    int deleteSelectedShapes() {
        int n = 0;
        const QList<QGraphicsItem *> selected = scene()->selectedItems();
        for (QGraphicsItem * item : selected) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (!shape) continue;
            if (shape == currentShape_) {
                currentShape_ = nullptr;
                pressed_ = false;
            }
            scene()->removeItem(shape);
            delete shape;
            ++n;
        }
        if (n > 0) {
            emit shapeEdited();
        }
        return n;
    }

    int togglePinSelected() {
        int n = 0;
        // If any selected shape is unpinned → pin all; otherwise unpin all.
        bool shouldPin = false;
        for (QGraphicsItem * item : scene()->selectedItems()) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (shape && shape->isFinished() && !shape->isPinned()) {
                shouldPin = true;
                break;
            }
        }
        for (QGraphicsItem * item : scene()->selectedItems()) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (!shape || !shape->isFinished()) continue;
            shape->setPinned(shouldPin);
            ++n;
        }
        if (n > 0) {
            emit shapeEdited();
        }
        return n;
    }

    void setSelectedPinned(bool pinned) {
        for (QGraphicsItem * item : scene()->selectedItems()) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (shape && shape->isFinished()) {
                shape->setPinned(pinned);
            }
        }
        emit shapeEdited();
    }

    void clearShapes() {
        QList<QGraphicsItem*> items = scene()->items();
        for (QGraphicsItem* item : items) {
            if (dynamic_cast<ShapeItem*>(item)) {
                scene()->removeItem(item);
                delete item;
            }
        }
        currentShape_ = nullptr;
    }

    /**
     * Undo one drawing step:
     * 1) If an in-progress shape exists, remove its last vertex (or cancel it).
     * 2) Else remove the topmost finished shape.
     * Returns true if something changed.
     */
    bool undoDrawingStep() {
        if (currentShape_) {
            if (!currentShape_->undoLastVertex()) {
                return false;
            }
            if (currentShape_->pointCount() == 0) {
                scene()->removeItem(currentShape_);
                delete currentShape_;
                currentShape_ = nullptr;
                pressed_ = false;
            }
            emit shapeEdited();
            return true;
        }

        // Prefer stacking-order topmost finished item (usually last drawn)
        ShapeItem * toRemove = nullptr;
        for (QGraphicsItem * item : scene()->items()) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (shape && shape->isFinished()) {
                toRemove = shape;
                break;
            }
        }
        if (!toRemove) {
            return false;
        }
        scene()->removeItem(toRemove);
        delete toRemove;
        emit shapeEdited();
        return true;
    }

    std::vector<QRectF> getShapes() const {
        std::vector<QRectF> shapes;
        QList<QGraphicsItem*> items = scene()->items();
        for (QGraphicsItem* item : items) {
            ShapeItem* shape = dynamic_cast<ShapeItem*>(item);
            if (shape && shape->isFinished()) {
                shapes.push_back(shape->boundingRect());
            }
        }
        return shapes;
    }

    /** Finish the in-progress shape if it already has enough geometry. */
    bool finishCurrentShapeIfPossible() {
        if (!currentShape_) {
            return false;
        }
        if (!currentShape_->finishIfReady()) {
            return false;
        }
        emit shapeFinished();
        currentShape_ = nullptr;
        return true;
    }

    /** Finalize every shape that has enough points (Save / Publish). */
    int finalizeAllReadyShapes() {
        int n = 0;
        if (finishCurrentShapeIfPossible()) {
            ++n;
        }
        for (ShapeItem * shape : getAllShapeItems()) {
            if (!shape->isFinished() && shape->finishIfReady()) {
                ++n;
            }
        }
        return n;
    }

    /** How many unfinished shapes are still on the scene. */
    int unfinishedShapeCount() const {
        int n = 0;
        for (QGraphicsItem * item : scene()->items()) {
            ShapeItem * shape = dynamic_cast<ShapeItem *>(item);
            if (shape && !shape->isFinished()) {
                ++n;
            }
        }
        return n;
    }

    std::vector<ShapeItem*> getFinishedShapeItems() const {
        std::vector<ShapeItem*> shapes;
        for (QGraphicsItem* item : scene()->items()) {
            ShapeItem* shape = dynamic_cast<ShapeItem*>(item);
            if (shape && shape->isFinished()) {
                shapes.push_back(shape);
            }
        }
        return shapes;
    }

    /** All keepout shapes, including the in-progress drawing. */
    std::vector<ShapeItem*> getAllShapeItems() const {
        std::vector<ShapeItem*> shapes;
        for (QGraphicsItem* item : scene()->items()) {
            ShapeItem* shape = dynamic_cast<ShapeItem*>(item);
            if (shape) {
                shapes.push_back(shape);
            }
        }
        return shapes;
    }

    ShapeItem* addFinishedShape(ShapeItem::ShapeType type, const QVector<QPointF>& scenePts,
                                QPointF sceneCenter = QPointF(), double sceneRadius = 0.0) {
        // New items have identity transform at origin, so scene == local coords.
        ShapeItem* shape = new ShapeItem(type);
        scene()->addItem(shape);
        if (type == ShapeItem::Circle) {
            shape->addPoint(sceneCenter);
            shape->addPoint(sceneCenter + QPointF(sceneRadius, 0.0));
        } else if (type == ShapeItem::Rectangle && scenePts.size() >= 4) {
            // 4 corners (TL,TR,BR,BL) preserve orientation after Save/Load.
            shape->applySceneGeometry(scenePts, QPointF(), 0.0, true);
        } else if (type == ShapeItem::Rectangle && scenePts.size() >= 2) {
            shape->addPoint(scenePts[0]);
            shape->addPoint(scenePts[1]);
        } else {
            for (const auto & sp : scenePts) {
                shape->addPoint(sp);
            }
            if (type == ShapeItem::Line || type == ShapeItem::Polygon) {
                shape->finish();
            }
        }
        // Loaded / restored shapes stay pinned until user unpins.
        if (shape->isFinished()) {
            shape->setPinned(true);
        }
        return shape;
    }

    /**
     * Keep shapes glued to geography when the tile mosaic origin/zoom changes.
     * sceneToLatLon / latLonToScene use the OLD then NEW tile grids respectively.
     */
    template <typename SceneToLatLon, typename LatLonToScene>
    void remapShapesForTileGridChange(SceneToLatLon sceneToLatLon, LatLonToScene latLonToScene) {
        struct Snapshot {
            ShapeItem * item;
            ShapeItem::ShapeType type;
            QVector<QPointF> scenePts;
            QPointF center;
            double radius;
            bool finished;
        };
        std::vector<Snapshot> snaps;
        for (ShapeItem * shape : getAllShapeItems()) {
            Snapshot s;
            s.item = shape;
            s.type = shape->shapeType();
            s.finished = shape->isFinished();
            if (s.type == ShapeItem::Circle) {
                s.center = shape->sceneCenter();
                s.radius = shape->sceneRadius();
            } else if (s.finished) {
                s.scenePts = shape->sceneVertices();
            } else {
                // In-progress vertices (sceneVertices() is empty until finished).
                s.scenePts = shape->allScenePoints();
            }
            snaps.push_back(s);
        }

        for (const Snapshot & s : snaps) {
            if (s.type == ShapeItem::Circle) {
                double lat = 0, lon = 0, lat2 = 0, lon2 = 0;
                if (!sceneToLatLon(s.center, lat, lon)) continue;
                if (!sceneToLatLon(s.center + QPointF(s.radius, 0.0), lat2, lon2)) continue;
                const QPointF c2 = latLonToScene(lat, lon);
                const QPointF r2 = latLonToScene(lat2, lon2);
                s.item->applySceneGeometry({}, c2, QLineF(c2, r2).length(), s.finished);
            } else {
                QVector<QPointF> mapped;
                mapped.reserve(s.scenePts.size());
                bool ok = true;
                for (const QPointF & sp : s.scenePts) {
                    double lat = 0, lon = 0;
                    if (!sceneToLatLon(sp, lat, lon)) {
                        ok = false;
                        break;
                    }
                    mapped.push_back(latLonToScene(lat, lon));
                }
                if (ok && (!mapped.isEmpty() || s.type == ShapeItem::Circle)) {
                    s.item->applySceneGeometry(mapped, QPointF(), 0.0, s.finished);
                }
            }
        }
    }

signals:
    void shapeFinished();
    void shapeEdited();  // undo / edit without completing a new shape
    void zoomDelta(double delta, QPointF mousePos);
    void centerChanged(double lat, double lon);
    void viewChanged();
    void panFinished(QPointF finalSceneCenter);

protected:
    void beginPan(const QPoint & pos) {
        panning_ = true;
        lastPanPoint_ = pos;
        setTransformationAnchor(QGraphicsView::AnchorViewCenter);
        setCursor(Qt::ClosedHandCursor);
    }

    void endPan() {
        if (!panning_) {
            return;
        }
        panning_ = false;
        setCursor(drawingMode_ ? Qt::CrossCursor : Qt::ArrowCursor);
        const QPointF finalCenter = mapToScene(viewport()->rect().center());
        emit panFinished(finalCenter);
    }

    void mousePressEvent(QMouseEvent* event) override {
        // Middle button: pan even while drawing (keeps currentShape_ intact).
        if (event->button() == Qt::MiddleButton) {
            beginPan(event->pos());
            event->accept();
            return;
        }
        if (!drawingMode_ && event->button() == Qt::LeftButton) {
            const QPointF scenePos = mapToScene(event->pos());
            ShapeItem * hit = findShapeAt(scenePos);
            if (hit) {
                // Edit mode: select shape (and move if unpinned). Empty map still pans.
                scene()->clearSelection();
                hit->setSelected(true);
                hit->setZValue(20);
                QGraphicsView::mousePressEvent(event);
                return;
            }
            scene()->clearSelection();
            beginPan(event->pos());
            event->accept();
            return;
        }
        if (drawingMode_) {
            QPointF scenePos = mapToScene(event->pos());

            // If the user clicked on an existing finished ShapeItem (e.g. one
            // showing corner handles), do NOT start a new shape. Let the
            // item receive the event so resize/rotate/move can begin.
            QGraphicsItem* hit = scene()->itemAt(scenePos, QTransform());
            ShapeItem* hitShape = dynamic_cast<ShapeItem*>(hit);
            if (hitShape && hitShape->isFinished()) {
                // Only forward the event to the item (so it can move/resize/rotate)
                // when the click actually hits the shape (or its handles). If the
                // click is within the item's bounding rect but outside the real
                // geometry, allow creating a new shape instead.
                if (hitShape->acceptsMoveAtScenePos(scenePos)) {
                    QGraphicsView::mousePressEvent(event);
                    return;
                }
                // else: fall through and allow drawing logic to proceed
            }

            // If the view is zoomed out too far, avoid creating new shapes
            // because handles become tiny and interaction is error-prone.
            qreal visualScale = transform().m11();
            if (visualScale < minVisualScaleForDrawing_) {
                qDebug() << "Drawing disabled at visual scale" << visualScale << "(<" << minVisualScaleForDrawing_ << ")";
                // ignore the click for creating shapes
                return;
            }

            if (event->button() == Qt::LeftButton) {
                pressed_ = true;
                if (currentShapeType_ == ShapeItem::Circle) {
                    if (!currentShape_) {
                        currentShape_ = new ShapeItem(ShapeItem::Circle);
                        scene()->addItem(currentShape_);
                        currentShape_->addPoint(scenePos); // set center
                    }
                } else if (currentShapeType_ == ShapeItem::Line ||
                           currentShapeType_ == ShapeItem::Rectangle) {
                    // Press = start point / first corner; drag sets end; release finishes.
                    // Construct with the explicit tool type (do not rely on a stale enum).
                    if (currentShape_) {
                        scene()->removeItem(currentShape_);
                        delete currentShape_;
                        currentShape_ = nullptr;
                    }
                    const ShapeItem::ShapeType tool =
                        (currentShapeType_ == ShapeItem::Line) ? ShapeItem::Line
                                                               : ShapeItem::Rectangle;
                    currentShape_ = new ShapeItem(tool);
                    scene()->addItem(currentShape_);
                    currentShape_->addPoint(scenePos);
                } else {
                    // Polygon: multi-click vertices, right-click / Enter to finish.
                    if (!currentShape_) {
                        currentShape_ = new ShapeItem(ShapeItem::Polygon);
                        scene()->addItem(currentShape_);
                    }
                    currentShape_->addPoint(scenePos);
                    if (currentShape_->isFinished()) {
                        emit shapeFinished();
                        currentShape_ = nullptr;
                    }
                }
            } else if (event->button() == Qt::RightButton) {
                // Right-click finishes Polygon only (Line/Rect finish on mouse release).
                if (currentShape_ && currentShapeType_ == ShapeItem::Polygon) {
                    if (currentShape_->finishIfReady()) {
                        emit shapeFinished();
                        currentShape_ = nullptr;
                    }
                }
            }
        }
        QGraphicsView::mousePressEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (panning_ &&
            (event->button() == Qt::MiddleButton ||
             (event->button() == Qt::LeftButton && !drawingMode_))) {
            endPan();
            event->accept();
            return;
        }

        if (drawingMode_ && event->button() == Qt::LeftButton) {
            pressed_ = false;
            if (currentShape_ && currentShapeType_ == ShapeItem::Circle) {
                currentShape_->finish();
                emit shapeFinished();
                currentShape_ = nullptr;
            } else if (currentShape_ &&
                       (currentShapeType_ == ShapeItem::Line ||
                        currentShapeType_ == ShapeItem::Rectangle)) {
                // Require a non-degenerate segment / rect.
                const bool ok = currentShape_->finishIfReady() &&
                    (currentShapeType_ != ShapeItem::Line || currentShape_->pointCount() >= 2) &&
                    (currentShapeType_ != ShapeItem::Rectangle || currentShape_->pointCount() >= 2);
                if (ok && currentShape_->pointCount() >= 2) {
                    // Reject zero-length drag (start == end).
                    const auto verts = currentShape_->allScenePoints();
                    bool degenerate = false;
                    if (verts.size() >= 2 &&
                        QLineF(verts.first(), verts.last()).length() < 1.0) {
                        degenerate = true;
                    }
                    if (degenerate) {
                        scene()->removeItem(currentShape_);
                        delete currentShape_;
                        currentShape_ = nullptr;
                    } else {
                        emit shapeFinished();
                        currentShape_ = nullptr;
                    }
                } else if (currentShape_) {
                    scene()->removeItem(currentShape_);
                    delete currentShape_;
                    currentShape_ = nullptr;
                }
            }
        }
        QGraphicsView::mouseReleaseEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (panning_) {
            // True grab-hand: move by viewport pixels on the scrollbars.
            const QPoint delta = event->pos() - lastPanPoint_;
            lastPanPoint_ = event->pos();
            if (QScrollBar * h = horizontalScrollBar()) {
                h->setValue(h->value() - delta.x());
            }
            if (QScrollBar * v = verticalScrollBar()) {
                v->setValue(v->value() - delta.y());
            }
            event->accept();
            return;
        }

        if (drawingMode_ && pressed_ && currentShape_) {
            QPointF scenePos = mapToScene(event->pos());
            if (currentShapeType_ == ShapeItem::Circle) {
                double radius = QLineF(currentShape_->getCenter(), scenePos).length();
                currentShape_->setRadius(radius);
            } else if (currentShapeType_ == ShapeItem::Line ||
                       currentShapeType_ == ShapeItem::Rectangle) {
                currentShape_->setEndpoint(scenePos);
            }
        }
        QGraphicsView::mouseMoveEvent(event);
    }

    void keyPressEvent(QKeyEvent* event) override {
        // Delete selected shape(s)
        if (event->key() == Qt::Key_Delete) {
            if (deleteSelectedShapes() > 0) {
                event->accept();
                return;
            }
        }
        // Ctrl+Z: undo last drawing step
        if (event->key() == Qt::Key_Z && event->modifiers() & Qt::ControlModifier) {
            if (undoDrawingStep()) {
                event->accept();
                return;
            }
        }
        // Backspace: delete selection in edit mode, else undo drawing vertex
        if (event->key() == Qt::Key_Backspace) {
            if (!drawingMode_ && !scene()->selectedItems().isEmpty()) {
                if (deleteSelectedShapes() > 0) {
                    event->accept();
                    return;
                }
            }
            if (undoDrawingStep()) {
                event->accept();
                return;
            }
        }
        // P: toggle pin on selected
        if (event->key() == Qt::Key_P && !drawingMode_) {
            if (togglePinSelected() > 0) {
                event->accept();
                return;
            }
        }
        if (event->key() == Qt::Key_Escape && currentShape_) {
            scene()->removeItem(currentShape_);
            delete currentShape_;
            currentShape_ = nullptr;
            pressed_ = false;
            emit shapeEdited();
            event->accept();
            return;
        }
        // Enter / Return finishes Polygon only.
        if ((event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) &&
            currentShape_ && currentShapeType_ == ShapeItem::Polygon) {
            if (finishCurrentShapeIfPossible()) {
                event->accept();
                return;
            }
        }
        QGraphicsView::keyPressEvent(event);
    }

    void wheelEvent(QWheelEvent* event) override {
        // Set zoom anchor to mouse position for centered zoom
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        // Emit a small fractional zoom delta for smoother zooming.
        // Typical wheel step is 120 units per notch. We convert that to
        // a small fractional value (e.g. 0.25 per notch) so the main window
        // can accumulate these and only change tile levels when thresholds
        // are crossed. This prevents jumping into "no imagery" quickly.
        int stepsInt = event->angleDelta().y() / 120; // integer division
        double rawSteps = event->angleDelta().y() / 120.0; // fractional
        double delta = 0.0;
        if (stepsInt != 0) {
            // clear integer notch detected
            delta = static_cast<double>(stepsInt);
        } else if (std::fabs(rawSteps) >= 0.5) {
            // Some mice produce non-120 but still large deltas; treat as a notch
            delta = (rawSteps > 0) ? 1.0 : -1.0;
        } else {
            // high-resolution or fractional devices: emit small fractional delta
            delta = rawSteps * 0.25; // small fractional
            if (qFuzzyIsNull(delta)) {
                // fallback for very small deltas
                delta = (event->angleDelta().y() > 0) ? 0.05 : -0.05;
            }
        }

        // Debug: log computed wheel steps and emitted delta so we can verify
        qDebug() << "wheelEvent stepsInt:" << stepsInt << "rawSteps:" << rawSteps << "angleDeltaY:" << event->angleDelta().y()
                 << "emitted delta:" << delta << "pos:" << event->position();
        emit zoomDelta(delta, event->position());
        event->accept();
    }

private:
    ShapeItem* currentShape_;
    bool drawingMode_;
    ShapeItem::ShapeType currentShapeType_;
    bool pressed_;
    bool panning_;
    QPoint lastPanPoint_;
};

/** True if 4 scene points form an axis-aligned (or near) rectangle. */
static bool isOrthogonalQuad(const QVector<QPointF> & pts) {
    if (pts.size() != 4) {
        return false;
    }
    auto nearZero = [](double v) { return std::abs(v) < 2.0; };  // ~2 px
    auto nearOrtho = [&](QPointF a, QPointF b) {
        return nearZero(a.x() * b.x() + a.y() * b.y());
    };
    const QPointF e0 = pts[1] - pts[0];
    const QPointF e1 = pts[2] - pts[1];
    const QPointF e2 = pts[3] - pts[2];
    const QPointF e3 = pts[0] - pts[3];
    if (QLineF(pts[0], pts[1]).length() < 1.0 || QLineF(pts[1], pts[2]).length() < 1.0) {
        return false;
    }
    // Adjacent edges roughly perpendicular, and opposite edges roughly parallel.
    if (!nearOrtho(e0, e1) || !nearOrtho(e1, e2)) {
        return false;
    }
    return nearZero(e0.x() * e2.y() - e0.y() * e2.x()) &&
           nearZero(e1.x() * e3.y() - e1.y() * e3.x());
}

class MainWindowWrapper : public QWidget {
    Q_OBJECT

public:
    MainWindowWrapper(QWidget *parent = nullptr)
    : QWidget(parent)
    , qnode_(new QNode(this))
    , tileLoader_(new TileMapLoader(this))
    , pgmAssist_(new PgmAssistController(this))
    {
        loadConfig();
        currentZoom_ = defaultZoom_;
        currentCenterLat_ = defaultCenterLat_;
        currentCenterLon_ = defaultCenterLon_;
        firstTileLoad_ = true;
        setupUI();
        setupConnections();
        refreshMapNameList();
        qnode_->start();
        if (pgmAssist_ && qnode_) {
            pgmAssist_->setNode(qnode_->node());
            pgmAssist_->attach(scene_, mapItem_, mapView_);
            pgmAssist_->setTileConverters(
                [this](const QPointF & scene, double & lat, double & lon) -> bool {
                    return sceneToLatLon(scene, lat, lon);
                },
                [this](double lat, double lon) -> QPointF {
                    return latLonToScene(lat, lon);
                },
                [this](const QPointF & scene, double & x, double & y, QString & err) -> bool {
                    double lat = 0, lon = 0;
                    if (!sceneToLatLon(scene, lat, lon)) {
                        err = QStringLiteral("Cannot convert scene→lat/lon (tile mosaic?)");
                        return false;
                    }
                    double lat0 = 0, lon0 = 0, yawRad = 0;
                    std::string e;
                    if (!readOriginLatLon(lat0, lon0, yawRad, e)) {
                        err = QString::fromStdString(e);
                        return false;
                    }
                    const auto p = latLonToMapMeters(lat, lon, lat0, lon0, yawRad);
                    x = p.x;
                    y = p.y;
                    return true;
                },
                [this](double x, double y, QPointF & scene, QString & err) -> bool {
                    double lat0 = 0, lon0 = 0, yawRad = 0;
                    std::string e;
                    if (!readOriginLatLon(lat0, lon0, yawRad, e)) {
                        err = QString::fromStdString(e);
                        return false;
                    }
                    if (currentMinTileX_ == INT_MAX) {
                        err = QStringLiteral("Load tile map first");
                        return false;
                    }
                    double lat = 0, lon = 0;
                    mapMetersToLatLon(x, y, lat0, lon0, yawRad, lat, lon);
                    scene = latLonToScene(lat, lon);
                    return true;
                });
            connect(pgmAssist_, &PgmAssistController::toolModeChanged, this, [this](bool exclusive) {
                if (exclusive) {
                    stopDrawing();
                }
            });
            connect(pgmAssist_, &PgmAssistController::statusMessage, this, [this](const QString & msg) {
                if (drawingToolLabel_) {
                    drawingToolLabel_->setText(msg);
                }
            });
        }
        // Timer to debounce pan updates before reloading tiles
        panDebounceTimer_ = new QTimer(this);
        panDebounceTimer_->setSingleShot(true);
        connect(panDebounceTimer_, &QTimer::timeout, this, [this]() {
            // PGM maps must not trigger online tile reloads after pan.
            if (!isPgmMap_) {
                loadOnlineTileMap();
            }
        });
        // Debounce viewport resize (fullscreen / maximize) so mosaic covers the new size
        resizeDebounceTimer_ = new QTimer(this);
        resizeDebounceTimer_->setSingleShot(true);
        connect(resizeDebounceTimer_, &QTimer::timeout, this, &MainWindowWrapper::onMapViewportResized);
        connect(tileLoader_, &TileMapLoader::mapLoaded, this, &MainWindowWrapper::onTileMapLoaded);

        if (mapView_ && mapView_->viewport()) {
            mapView_->viewport()->installEventFilter(this);
        }

        // Auto-load online tiles once the window has a real viewport size
        QTimer::singleShot(0, this, [this]() {
            loadOnlineTileMap();
        });
    }

    ~MainWindowWrapper() {
        delete qnode_;
        delete tileLoader_;
    }

protected:
    bool eventFilter(QObject * obj, QEvent * event) override {
        if (mapView_ && obj == mapView_->viewport() && event->type() == QEvent::Resize) {
            if (resizeDebounceTimer_) {
                resizeDebounceTimer_->start(120);
            }
        }
        return QWidget::eventFilter(obj, event);
    }

    void showEvent(QShowEvent * event) override {
        QWidget::showEvent(event);
        // After first show / maximize, viewport size may change — refill mosaic.
        if (resizeDebounceTimer_) {
            resizeDebounceTimer_->start(200);
        }
    }

public:
    void loadConfig() {
        // Load default configuration from config/default_config.yaml
        QString configPath = QDir(QApplication::applicationDirPath()).absoluteFilePath("../config/default_config.yaml");
        QFile configFile(configPath);
        if (!configFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning() << "Cannot open config file:" << configPath << "Using default values.";
            defaultCenterLat_ = 30.59170598225533;
            defaultCenterLon_ = 104.08352549614284;
            defaultZoom_ = 18;
            defaultTileUrl_ = "https://t0.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={level}&TILEROW={y}&TILECOL={x}&tk=f6cfa9d6a754fb135ba600ff23549c69";
            return;
        }

        QTextStream in(&configFile);
        QString content = in.readAll();
        configFile.close();

        // Simple YAML parsing
        QStringList lines = content.split('\n');
        for (const QString& line : lines) {
            if (line.startsWith("default_center_lat:")) {
                defaultCenterLat_ = line.mid(19).trimmed().toDouble();
            } else if (line.startsWith("default_center_lon:")) {
                defaultCenterLon_ = line.mid(19).trimmed().toDouble();
            } else if (line.startsWith("default_zoom:")) {
                defaultZoom_ = line.mid(13).trimmed().toInt();
            } else if (line.startsWith("default_tile_url:")) {
                // start after the ':' character (17 characters)
                QString url = line.mid(17).trimmed();
                // remove surrounding quotes if present and trim again
                if (url.startsWith('"') && url.endsWith('"')) {
                    url = url.mid(1, url.length() - 2).trimmed();
                } else {
                    url.remove('"');
                    url = url.trimmed();
                }
                defaultTileUrl_ = url;
            }
        }

        qDebug() << "Config loaded successfully:" << QString::number(defaultCenterLat_, 'f', 14) << QString::number(defaultCenterLon_, 'f', 14) << defaultZoom_;
    }

public slots:
    void onGpsUpdate(double lat, double lon) {
        gpsLatEdit_->setText(QString::number(lat, 'f', 10));
        gpsLonEdit_->setText(QString::number(lon, 'f', 10));
    }

    void onViewChanged(const QPointF &finalSceneCenter) {
        // Called once when panning finishes. Keep continuous tile coords so
        // the subsequent tile reload can restore the exact geographic center.
        if (isPgmMap_) {
            // Local PGM: pan/zoom only the view — do not reload online tiles.
            if (panDebounceTimer_) {
                panDebounceTimer_->stop();
            }
            updateInflationPreview();
            return;
        }
        if (currentMinTileX_ == INT_MAX) return; // no tiles loaded yet
        const int z = mosaicZoom();
        const double tileX = finalSceneCenter.x() / 256.0 + currentMinTileX_;
        const double tileY = finalSceneCenter.y() / 256.0 + currentMinTileY_;
        currentCenterLon_ = tilex2lon_d(tileX, z);
        currentCenterLat_ = tiley2lat_d(tileY, z);
        qDebug() << "onViewChanged(): updated center to" << currentCenterLat_ << currentCenterLon_
                 << "tile:" << tileX << tileY << "z=" << z;
        updateInflationPreview();
        // Debounce tile reload only after the finger has stopped; view itself
        // already stopped on mouse release.
        if (panDebounceTimer_) panDebounceTimer_->start(300);
    }

    void onShapeFinished() {
        updateProhibitionAreas();
        updateInflationPreview();
    }

    /** meters/pixel in scene coordinates (for inflation preview). */
    double metersPerPixelAtView() const {
        if (isPgmMap_) {
            return (mapResolution_ > 1e-9) ? mapResolution_ : 1.0;
        }
        const int z = (displayedTileZoom_ >= 1) ? displayedTileZoom_ : currentZoom_;
        if (z < 1) {
            return 1.0;
        }
        const double latRad = currentCenterLat_ * M_PI / 180.0;
        return std::cos(latRad) * 2.0 * M_PI * 6378137.0 / (256.0 * std::pow(2.0, z));
    }

    void updateInflationPreview() {
        if (!mapView_ || !inscribedRadiusSpin_ || !inflationRadiusSpin_ || !showInflationChk_) {
            return;
        }
        mapView_->refreshInflationPreview(
            showInflationChk_->isChecked(),
            inscribedRadiusSpin_->value(),
            inflationRadiusSpin_->value(),
            metersPerPixelAtView());
    }

    void onZoomChanged(double delta, QPointF mousePos) {
        qDebug() << "onZoomChanged(): delta=" << delta << " zoomAcc=" << zoomAccumulator_
                 << " curZoom=" << currentZoom_ << " displayedZoom=" << displayedTileZoom_
                 << " visualScale=" << currentVisualScale_;
        if (isPgmMap_) {
            qreal factor = std::pow(1.1, delta);
            currentVisualScale_ *= factor;
            mapView_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            mapView_->setTransform(QTransform::fromScale(currentVisualScale_, currentVisualScale_));
            emit mapView_->centerChanged(currentCenterLat_, currentCenterLon_);
            updateInflationPreview();
            return;
        }

        const int MAX_TILE_ZOOM = 18;
        const int gridZoom = (displayedTileZoom_ >= 1) ? displayedTileZoom_ : currentZoom_;

        zoomAccumulator_ += delta;

        int stepChange = 0;
        if (zoomAccumulator_ >= 1.0) {
            stepChange = static_cast<int>(std::floor(zoomAccumulator_));
            zoomAccumulator_ -= stepChange;
        } else if (zoomAccumulator_ <= -1.0) {
            stepChange = static_cast<int>(std::ceil(zoomAccumulator_));
            zoomAccumulator_ -= stepChange;
        }

        if (stepChange != 0) {
            qDebug() << "onZoomChanged(): stepChange=" << stepChange << " beforeZoom=" << currentZoom_;
            // Update center from mouse using the CURRENTLY DISPLAYED tile grid zoom.
            if (!mousePos.isNull() && currentMinTileX_ != INT_MAX) {
                QPointF scenePos = mapView_->mapToScene(mousePos.toPoint());
                const double tileX = scenePos.x() / 256.0 + currentMinTileX_;
                const double tileY = scenePos.y() / 256.0 + currentMinTileY_;
                currentCenterLon_ = tilex2lon_d(tileX, gridZoom);
                currentCenterLat_ = tiley2lat_d(tileY, gridZoom);
            }

            int intendedZoom = currentZoom_ + stepChange;
            int clampedZoom = intendedZoom;
            if (clampedZoom < 1) clampedZoom = 1;
            if (clampedZoom > MAX_TILE_ZOOM) clampedZoom = MAX_TILE_ZOOM;

            if (clampedZoom != currentZoom_) {
                currentZoom_ = clampedZoom;
                qDebug() << "onZoomChanged(): afterZoom=" << currentZoom_
                         << " (geo remap via lat/lon; reset visual scale)";
                zoomAccumulator_ = 0.0;
                // Integer tile zoom already changes geographic scale (~2x).
                // Reset view scale so shapes/tiles stay consistent.
                currentVisualScale_ = 1.0;
                firstTileLoad_ = false;
                loadOnlineTileMap();
                return;
            }

            // At zoom boundary: visual-only zoom (no new tiles).
            qDebug() << "onZoomChanged(): at zoom boundary (" << currentZoom_
                     << ") - applying visual scale only";
            zoomAccumulator_ = 0.0;
            mapView_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            qreal visualFactor = std::pow(1.2, static_cast<double>(stepChange));
            currentVisualScale_ *= visualFactor;
            // Clamp runaway visual zoom at boundaries
            if (currentVisualScale_ < 0.25) currentVisualScale_ = 0.25;
            if (currentVisualScale_ > 8.0) currentVisualScale_ = 8.0;
            mapView_->setTransform(QTransform::fromScale(currentVisualScale_, currentVisualScale_));
            return;
        }

        // Fractional wheel zoom between tile levels: temporary visual scale only.
        mapView_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        qreal visualFactor = std::pow(1.2, delta);
        currentVisualScale_ *= visualFactor;
        mapView_->setTransform(QTransform::fromScale(currentVisualScale_, currentVisualScale_));
    }

private slots:
    void loadMap() {
        const QString type = mapTypeCombo_->currentData().toString();
        const bool wantPgm = (type == QLatin1String("Local PGM Map") ||
            mapTypeCombo_->currentText().contains(QStringLiteral("PGM")));
        if (wantPgm) {
            loadLocalPgmMap();
        } else {
            // User clicked「加载地图」to leave PGM: drop previous keepouts, enable tiles.
            if (isPgmMap_) {
                clearShapes();
                if (pgmAssist_) {
                    pgmAssist_->clearPgmMap(false);
                }
                isPgmMap_ = false;
            }
            loadOnlineTileMap();
        }
    }

    void loadLocalPgmMap() {
        QString pgmFileName = QFileDialog::getOpenFileName(this, "Select PGM file", "", "PGM files (*.pgm)");
        if (pgmFileName.isEmpty()) return;

        // Try to find corresponding YAML file in the same directory
        QFileInfo pgmInfo(pgmFileName);
        QString yamlFileName = pgmInfo.absoluteDir().absoluteFilePath(pgmInfo.baseName() + ".yaml");
        if (!QFile::exists(yamlFileName)) {
            yamlFileName = pgmInfo.absoluteDir().absoluteFilePath(pgmInfo.baseName() + ".yml");
        }
        if (!QFile::exists(yamlFileName)) {
            // Try in the map folder
            QString mapDir = QDir(QApplication::applicationDirPath()).absoluteFilePath("../../src/map_coordinates_edit_gui/map");
            yamlFileName = QDir(mapDir).absoluteFilePath(pgmInfo.baseName() + ".yaml");
            if (!QFile::exists(yamlFileName)) {
                yamlFileName = QDir(mapDir).absoluteFilePath(pgmInfo.baseName() + ".yml");
            }
        }

        if (!QFile::exists(yamlFileName)) {
            QMessageBox::warning(this, "Error", "Cannot find corresponding YAML file for " + pgmFileName);
            return;
        }

        // Parse YAML file
        QFile yamlFile(yamlFileName);
        if (!yamlFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "Error", "Cannot open YAML file");
            return;
        }

        QTextStream in(&yamlFile);
        QString content = in.readAll();
        yamlFile.close();

        // Simple YAML parsing (basic implementation)
        double resolution = 0.0;
        double originX = 0.0, originY = 0.0, originYaw = 0.0;
        QString yamlImage;
        QString yamlMode;
        int yamlNegate = 0;
        double occupiedThresh = 0.0;
        double freeThresh = 0.0;

        QStringList lines = content.split('\n');
        for (const QString& line : lines) {
            const QString trimmed = line.trimmed();
            if (trimmed.startsWith("resolution:")) {
                resolution = trimmed.mid(11).trimmed().toDouble();
            } else if (trimmed.startsWith("origin:")) {
                QString originStr = trimmed.mid(7).trimmed();
                originStr.remove('[').remove(']');
                QStringList parts = originStr.split(',');
                if (parts.size() >= 2) {
                    originX = parts[0].trimmed().toDouble();
                    originY = parts[1].trimmed().toDouble();
                }
                if (parts.size() >= 3) {
                    originYaw = parts[2].trimmed().toDouble();
                }
            } else if (trimmed.startsWith("image:")) {
                yamlImage = trimmed.mid(6).trimmed();
                if ((yamlImage.startsWith('"') && yamlImage.endsWith('"')) ||
                    (yamlImage.startsWith('\'') && yamlImage.endsWith('\''))) {
                    yamlImage = yamlImage.mid(1, yamlImage.size() - 2);
                }
            } else if (trimmed.startsWith("mode:")) {
                yamlMode = trimmed.mid(5).trimmed();
            } else if (trimmed.startsWith("negate:")) {
                yamlNegate = trimmed.mid(7).trimmed().toInt();
            } else if (trimmed.startsWith("occupied_thresh:")) {
                occupiedThresh = trimmed.mid(16).trimmed().toDouble();
            } else if (trimmed.startsWith("free_thresh:")) {
                freeThresh = trimmed.mid(12).trimmed().toDouble();
            }
        }

        // Load PGM image (prefer robust P2/P5 parser)
        QImage editable;
        if (!pgm_io::loadPGM(pgmFileName, editable)) {
            QPixmap fallback(pgmFileName);
            if (fallback.isNull()) {
                QMessageBox::warning(this, "Error", "Cannot load PGM image: " + pgmFileName);
                return;
            }
            editable = fallback.toImage().convertToFormat(QImage::Format_Grayscale8);
        }
        QPixmap pixmap = QPixmap::fromImage(editable);
        if (!pixmap.isNull()) {
            // Stop any pending tile reload from a previous pan on the tile map.
            if (panDebounceTimer_) {
                panDebounceTimer_->stop();
            }
            // Keepouts belong to the previous map — clear when switching base map.
            clearShapes();
            isPgmMap_ = true;
            currentVisualScale_ = 1.0;
            zoomAccumulator_ = 0.0;
            // Invalidate tile mosaic bookkeeping so a late mapLoaded cannot remap shapes.
            currentMinTileX_ = INT_MAX;
            currentMinTileY_ = INT_MAX;
            displayedTileZoom_ = -1;
            mapItem_->setPixmap(pixmap);
            mapItem_->setZValue(0);
            mapItem_->setPos(0, 0);
            if (scene_) {
                scene_->setSceneRect(pixmap.rect());
            }
            mapResolution_ = resolution;
            mapOriginX_ = originX;
            mapOriginY_ = originY;
            mapOriginYaw_ = originYaw;
            mapHeightPx_ = pixmap.height();
            mapWidthPx_ = pixmap.width();
            currentPgmPath_ = pgmFileName;
            currentPgmYamlPath_ = yamlFileName;
            pgmYamlImage_ = yamlImage;
            pgmYamlMode_ = yamlMode;
            pgmYamlNegate_ = yamlNegate;
            pgmOccupiedThresh_ = occupiedThresh;
            pgmFreeThresh_ = freeThresh;
            mapView_->resetTransform();
            mapView_->fitInView(mapItem_, Qt::KeepAspectRatio);
            mapView_->centerOn(mapItem_);
            if (pgmAssist_) {
                pgmAssist_->setPgmMap(editable, pgmFileName, resolution, originX, originY);
            }
            refreshCoordPanelForMapMode();
            updateInflationPreview();
            qDebug() << "Loaded PGM map:" << pgmFileName
                     << "yaml:" << yamlFileName
                     << "resolution:" << mapResolution_
                     << "origin:[" << mapOriginX_ << "," << mapOriginY_ << "," << mapOriginYaw_ << "]"
                     << "size:" << mapWidthPx_ << "x" << mapHeightPx_;
        } else {
            QMessageBox::warning(this, "Error", "Cannot load PGM image: " + pgmFileName);
        }
    }

    void loadOnlineTileMap() {
        QString url = tileUrlEdit_->text();
        if (url.isEmpty()) {
            QMessageBox::warning(this, "Error", "Please enter a tile URL");
            return;
        }

        qDebug() << "Tile URL text box contents:" << url;
        url = url.trimmed();
        // remove surrounding quotes if present
        if (url.startsWith('"') && url.endsWith('"')) {
            url = url.mid(1, url.length() - 2);
        }

        // Mark as tile map mode
        if (isPgmMap_ && pgmAssist_) {
            pgmAssist_->clearPgmMap(false);
        }
        isPgmMap_ = false;
        refreshCoordPanelForMapMode();
        // Use current center coordinates
        double centerLat = currentCenterLat_;
        double centerLon = currentCenterLon_;

        qDebug() << "Calling loadTileMap with zoom,center:" << currentZoom_ << centerLat << centerLon;
        // Cover the viewport in *scene* pixels (account for view scale).
        int tileSize = 256;
        QSize vp = mapView_->viewport()->size();
        const double scale = std::max(0.05, std::abs(mapView_->transform().m11()));
        // How much scene width/height is visible
        const double sceneW = static_cast<double>(vp.width()) / scale;
        const double sceneH = static_cast<double>(vp.height()) / scale;
        int tilesHalfX = static_cast<int>(std::ceil(sceneW / static_cast<double>(tileSize) / 2.0));
        int tilesHalfY = static_cast<int>(std::ceil(sceneH / static_cast<double>(tileSize) / 2.0));
        // +2 margin so maximize/fullscreen does not leave a strip of empty scene
        int radius = std::max(2, std::max(tilesHalfX, tilesHalfY) + 2);
        qDebug() << "Computed tile radius:" << radius << "for viewport" << vp
                 << "scale" << scale << "sceneNeed" << sceneW << "x" << sceneH;
        tileLoader_->loadTileMap(url, currentZoom_, centerLat, centerLon, tileSize, radius);
        lastTileViewport_ = vp;
    }

    /** After maximize/fullscreen/sidebar toggle: refill tiles if mosaic no longer covers the view. */
    void onMapViewportResized() {
        if (isPgmMap_ || !mapView_ || !mapItem_) {
            return;
        }
        const QSize vp = mapView_->viewport()->size();
        if (vp.width() < 32 || vp.height() < 32) {
            return;
        }

        // Keep geographic center under the viewport center
        if (currentMinTileX_ != INT_MAX && mosaicZoom() >= 1) {
            const QPointF sc = mapView_->mapToScene(vp.width() / 2, vp.height() / 2);
            const int z = mosaicZoom();
            const double tileX = sc.x() / 256.0 + currentMinTileX_;
            const double tileY = sc.y() / 256.0 + currentMinTileY_;
            currentCenterLon_ = tilex2lon_d(tileX, z);
            currentCenterLat_ = tiley2lat_d(tileY, z);
        }

        const QRectF mosaic = mapItem_->sceneBoundingRect();
        const QRectF visible = mapView_->mapToScene(mapView_->viewport()->rect()).boundingRect();
        // True when the view shows empty scene past the mosaic (typical right/bottom blank).
        const QRectF inset = visible.adjusted(4, 4, -4, -4);
        const bool uncovered = mosaic.isEmpty() || !mosaic.contains(inset);
        const bool grewALot = lastTileViewport_.isValid() &&
            (vp.width() > lastTileViewport_.width() + 64 ||
             vp.height() > lastTileViewport_.height() + 64);

        if (uncovered || grewALot || lastTileViewport_.isEmpty()) {
            qDebug() << "onMapViewportResized(): reloading tiles"
                     << "vp" << vp << "uncovered" << uncovered << "grew" << grewALot;
            firstTileLoad_ = false;  // avoid fitInView jump; keep center
            loadOnlineTileMap();
        }
    }

    void onTileMapLoaded() {
        // Ignore late tile callbacks after the user switched to a local PGM map.
        if (isPgmMap_) {
            return;
        }
        // If tileLoader returned no tiles for this zoom, attempt to fallback
        if (tileLoader_->getTiles().isEmpty()) {
            qWarning() << "No tiles returned for zoom" << currentZoom_ << ", trying lower zoom";
            if (currentZoom_ > 1) {
                currentZoom_ = std::max(1, currentZoom_ - 1);
                // Prevent aggressive re-fit on fallback
                firstTileLoad_ = true;
                loadOnlineTileMap();
                return;
            }
        }

        QPixmap pixmap = tileLoader_->getMapPixmap();
        if (!pixmap.isNull()) {
            const int oldMinX = currentMinTileX_;
            const int oldMinY = currentMinTileY_;
            // CRITICAL: use the zoom of the mosaic currently on screen, NOT currentZoom_
            // (currentZoom_ may already have been advanced before this reload).
            const int oldZoom = (displayedTileZoom_ >= 1) ? displayedTileZoom_ : currentZoom_;
            const bool hadOldGrid = (oldMinX != INT_MAX && oldMinY != INT_MAX && oldZoom >= 1);

            // Capture shape geography in the OLD tile frame before replacing the mosaic.
            struct GeoShape {
                ShapeItem * item{nullptr};
                ShapeItem::ShapeType type{ShapeItem::Line};
                std::vector<std::pair<double, double>> latlon;  // vertices or circle center
                std::pair<double, double> rimLatLon{0, 0};       // circle rim
                bool finished{true};
                bool valid{false};
            };
            std::vector<GeoShape> geoShapes;
            if (hadOldGrid) {
                auto sceneToLatLonOld = [&](const QPointF & scenePt, double & lat, double & lon) -> bool {
                    const double tileX = scenePt.x() / 256.0 + oldMinX;
                    const double tileY = scenePt.y() / 256.0 + oldMinY;
                    lon = tilex2lon_d(tileX, oldZoom);
                    lat = tiley2lat_d(tileY, oldZoom);
                    return true;
                };
                if (pgmAssist_) {
                    pgmAssist_->beginTileRemap(sceneToLatLonOld);
                }
                for (ShapeItem * shape : mapView_->getAllShapeItems()) {
                    GeoShape g;
                    g.item = shape;
                    g.type = shape->shapeType();
                    g.finished = shape->isFinished();
                    if (g.type == ShapeItem::Circle) {
                        double lat = 0, lon = 0, lat2 = 0, lon2 = 0;
                        if (!sceneToLatLonOld(shape->sceneCenter(), lat, lon)) continue;
                        if (!sceneToLatLonOld(shape->sceneCenter() + QPointF(shape->sceneRadius(), 0.0), lat2, lon2)) continue;
                        g.latlon.push_back({lat, lon});
                        g.rimLatLon = {lat2, lon2};
                        g.valid = true;
                    } else {
                        const QVector<QPointF> pts = g.finished
                            ? shape->sceneVertices()
                            : shape->allScenePoints();
                        g.valid = !pts.isEmpty();
                        for (const QPointF & sp : pts) {
                            double lat = 0, lon = 0;
                            if (!sceneToLatLonOld(sp, lat, lon)) {
                                g.valid = false;
                                break;
                            }
                            g.latlon.push_back({lat, lon});
                        }
                    }
                    if (g.valid) geoShapes.push_back(g);
                }
                qDebug() << "onTileMapLoaded(): capturing" << geoShapes.size()
                         << "shapes from grid z=" << oldZoom
                         << "min=(" << oldMinX << "," << oldMinY << ")"
                         << "→ new z=" << currentZoom_;
            }

            mapItem_->setPixmap(pixmap);
            mapItem_->setZValue(0);
            mapItem_->setPos(0, 0);
            scene_->setSceneRect(0, 0, pixmap.width(), pixmap.height());
            mapView_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            mapView_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

            currentMinTileX_ = INT_MAX;
            currentMinTileY_ = INT_MAX;
            int maxX = INT_MIN, maxY = INT_MIN;
            for (auto it = tileLoader_->getTiles().constBegin(); it != tileLoader_->getTiles().constEnd(); ++it) {
                currentMinTileX_ = std::min(currentMinTileX_, it.key().first);
                currentMinTileY_ = std::min(currentMinTileY_, it.key().second);
                maxX = std::max(maxX, it.key().first);
                maxY = std::max(maxY, it.key().second);
            }
            displayedTileZoom_ = currentZoom_;

            // Re-project path + keepout shapes into the NEW tile scene via lat/lon.
            auto latLonToSceneNew = [&](double lat, double lon) -> QPointF {
                const double tileX = lon2tilex_d(lon, currentZoom_);
                const double tileY = lat2tiley_d(lat, currentZoom_);
                return QPointF((tileX - currentMinTileX_) * 256.0,
                               (tileY - currentMinTileY_) * 256.0);
            };
            if (pgmAssist_ && currentMinTileX_ != INT_MAX) {
                pgmAssist_->endTileRemap(latLonToSceneNew);
                pgmAssist_->setTileMapActive();
            }

            if (!geoShapes.empty() && currentMinTileX_ != INT_MAX) {
                for (const GeoShape & g : geoShapes) {
                    if (!g.item) continue;
                    if (g.type == ShapeItem::Circle && !g.latlon.empty()) {
                        const QPointF c = latLonToSceneNew(g.latlon[0].first, g.latlon[0].second);
                        const QPointF r = latLonToSceneNew(g.rimLatLon.first, g.rimLatLon.second);
                        g.item->applySceneGeometry({}, c, QLineF(c, r).length(), g.finished);
                    } else {
                        QVector<QPointF> pts;
                        for (const auto & ll : g.latlon) {
                            pts.push_back(latLonToSceneNew(ll.first, ll.second));
                        }
                        g.item->applySceneGeometry(pts, QPointF(), 0.0, g.finished);
                    }
                }
                updateProhibitionAreas();
            }

            if (firstTileLoad_) {
                mapView_->resetTransform();
                mapView_->fitInView(mapItem_, Qt::KeepAspectRatioByExpanding);
                currentVisualScale_ = mapView_->transform().m11();
                if (currentVisualScale_ <= 0.0) currentVisualScale_ = 1.0;
                mapView_->centerOn(mapItem_);
                firstTileLoad_ = false;
            } else {
                mapView_->setTransform(QTransform::fromScale(currentVisualScale_, currentVisualScale_));
                const double tileX = lon2tilex_d(currentCenterLon_, currentZoom_);
                const double tileY = lat2tiley_d(currentCenterLat_, currentZoom_);
                const QPointF sceneCenter(
                    (tileX - currentMinTileX_) * 256.0,
                    (tileY - currentMinTileY_) * 256.0);
                mapView_->centerOn(sceneCenter);
            }
            qDebug() << "onTileMapLoaded(): pixmap size=" << pixmap.size()
                     << " currentMinTileX=" << currentMinTileX_
                     << " currentMinTileY=" << currentMinTileY_
                     << " displayedZoom=" << displayedTileZoom_
                     << " visualScale=" << currentVisualScale_;
            updateInflationPreview();
        }
    }

    void startDrawing() {
        const ShapeItem::ShapeType type = selectedShapeTool();
        mapView_->setDrawingMode(true, type);
        mapView_->setFocus(Qt::OtherFocusReason);
        syncToolButtonsToShape(type);
        updateDrawingToolLabel();
        qDebug() << "Start Drawing tool=" << shapeToolName(type)
                 << "enum" << static_cast<int>(type);
    }

    void onShapeToolChanged() {
        if (!mapView_) {
            return;
        }
        if (selectToolBtn_ && selectToolBtn_->isChecked()) {
            mapView_->setDrawingMode(false);
            updateDrawingToolLabel();
            return;
        }
        const ShapeItem::ShapeType type = selectedShapeTool();
        lastShapeTool_ = type;
        mapView_->setDrawingMode(true, type);
        mapView_->setFocus(Qt::OtherFocusReason);
        updateDrawingToolLabel();
        qDebug() << "Shape tool changed to" << shapeToolName(type)
                 << "enum" << static_cast<int>(type);
    }

    ShapeItem::ShapeType selectedShapeTool() const {
        if (rectToolBtn_ && rectToolBtn_->isChecked()) return ShapeItem::Rectangle;
        if (circleToolBtn_ && circleToolBtn_->isChecked()) return ShapeItem::Circle;
        if (polygonToolBtn_ && polygonToolBtn_->isChecked()) return ShapeItem::Polygon;
        if (lineToolBtn_ && lineToolBtn_->isChecked()) return ShapeItem::Line;
        return lastShapeTool_;
    }

    static QString shapeToolName(ShapeItem::ShapeType t) {
        switch (t) {
            case ShapeItem::Line: return QStringLiteral("线段");
            case ShapeItem::Rectangle: return QStringLiteral("矩形");
            case ShapeItem::Circle: return QStringLiteral("圆形");
            case ShapeItem::Polygon: return QStringLiteral("多边形");
        }
        return QStringLiteral("?");
    }

    void syncToolButtonsToShape(ShapeItem::ShapeType type) {
        if (!lineToolBtn_) return;
        lastShapeTool_ = type;
        QSignalBlocker b0(selectToolBtn_);
        QSignalBlocker b1(lineToolBtn_);
        QSignalBlocker b2(rectToolBtn_);
        QSignalBlocker b3(circleToolBtn_);
        QSignalBlocker b4(polygonToolBtn_);
        selectToolBtn_->setChecked(false);
        lineToolBtn_->setChecked(type == ShapeItem::Line);
        rectToolBtn_->setChecked(type == ShapeItem::Rectangle);
        circleToolBtn_->setChecked(type == ShapeItem::Circle);
        polygonToolBtn_->setChecked(type == ShapeItem::Polygon);
        syncChromeFromSidebar();
    }

    void syncToolButtonsToSelect() {
        if (!selectToolBtn_) return;
        QSignalBlocker b0(selectToolBtn_);
        QSignalBlocker b1(lineToolBtn_);
        QSignalBlocker b2(rectToolBtn_);
        QSignalBlocker b3(circleToolBtn_);
        QSignalBlocker b4(polygonToolBtn_);
        selectToolBtn_->setChecked(true);
        lineToolBtn_->setChecked(false);
        rectToolBtn_->setChecked(false);
        circleToolBtn_->setChecked(false);
        polygonToolBtn_->setChecked(false);
        syncChromeFromSidebar();
    }

    void syncChromeFromSidebar() {
        auto setChrome = [](QToolButton * btn, bool on) {
            if (!btn) return;
            QSignalBlocker b(btn);
            btn->setChecked(on);
        };
        setChrome(chromeSelectBtn_, selectToolBtn_ && selectToolBtn_->isChecked());
        setChrome(chromeLineBtn_, lineToolBtn_ && lineToolBtn_->isChecked());
        setChrome(chromeRectBtn_, rectToolBtn_ && rectToolBtn_->isChecked());
        setChrome(chromeCircleBtn_, circleToolBtn_ && circleToolBtn_->isChecked());
        setChrome(chromePolyBtn_, polygonToolBtn_ && polygonToolBtn_->isChecked());
    }

    void updateDrawingToolLabel() {
        if (!drawingToolLabel_) {
            return;
        }
        const bool on = mapView_ && mapView_->isDrawingMode();
        if (!on) {
            drawingToolLabel_->setText(
                QStringLiteral("选择模式：单击图形选中（蓝色高亮）。"
                               "解锁后可拖动/改尺寸；钉住锁定；Delete 删除。空白处拖动画布。"));
            return;
        }
        const auto t = selectedShapeTool();
        QString tip;
        switch (t) {
            case ShapeItem::Polygon:
                tip = QStringLiteral("左键加点，右键或 Enter 结束");
                break;
            case ShapeItem::Line:
                tip = QStringLiteral("拖拽绘制线段，右键可结束折线");
                break;
            default:
                tip = QStringLiteral("按住拖拽绘制");
                break;
        }
        drawingToolLabel_->setText(
            QStringLiteral("绘制中：%1 — %2").arg(shapeToolName(t), tip));
    }

    void toggleSidebar() {
        if (!sidebarFrame_) return;
        sidebarVisible_ = !sidebarVisible_;
        sidebarFrame_->setVisible(sidebarVisible_);
        if (drawerToggleBtn_) {
            drawerToggleBtn_->setText(sidebarVisible_
                ? QStringLiteral("◀ 收起")
                : QStringLiteral("▶ 菜单"));
            drawerToggleBtn_->setToolTip(sidebarVisible_
                ? QStringLiteral("收起侧边栏")
                : QStringLiteral("展开侧边栏"));
        }
    }

    void zoomIn() {
        onZoomChanged(1.0, QPointF());
    }

    void zoomOut() {
        onZoomChanged(-1.0, QPointF());
    }

    void resetZoom() {
        currentZoom_ = defaultZoom_;
        currentVisualScale_ = 1.0;
        zoomAccumulator_ = 0.0;
        // force an initial fit on next load
        firstTileLoad_ = true;
        if (isPgmMap_) {
            mapView_->resetTransform();
            mapView_->fitInView(mapItem_, Qt::KeepAspectRatio);
            mapView_->centerOn(mapItem_);
        } else {
            mapView_->resetTransform();
            loadOnlineTileMap();
        }
    }

    void stopDrawing() {
        mapView_->setDrawingMode(false);
        syncToolButtonsToSelect();
        updateDrawingToolLabel();
    }

    void clearShapes() {
        mapView_->clearShapes();
        updateProhibitionAreas();
    }

    void undoDrawing() {
        if (mapView_->undoDrawingStep()) {
            updateProhibitionAreas();
        }
    }

    void deleteSelectedShapes() {
        if (mapView_->isDrawingMode()) {
            QMessageBox::information(this, QStringLiteral("删除"),
                QStringLiteral("请先切换到「选择」模式，再单击图形后删除。"));
            return;
        }
        const int n = mapView_->deleteSelectedShapes();
        if (n == 0) {
            QMessageBox::information(this, QStringLiteral("删除"),
                QStringLiteral("未选中图形。\n选择模式 → 单击图形（蓝色高亮）→ 删除。"));
            return;
        }
        updateProhibitionAreas();
    }

    void pinSelectedShapes() {
        if (mapView_->isDrawingMode()) {
            QMessageBox::information(this, QStringLiteral("钉住"),
                QStringLiteral("请先切换到「选择」模式，再选中要锁定的图形。"));
            return;
        }
        if (mapView_->scene()->selectedItems().isEmpty()) {
            QMessageBox::information(this, QStringLiteral("钉住"),
                QStringLiteral("请先选中图形（选择模式 → 单击图形）。\n"
                               "红色钉 = 已锁定；解锁后可移动/改尺寸。"));
            return;
        }
        mapView_->setSelectedPinned(true);
    }

    void unpinSelectedShapes() {
        if (mapView_->isDrawingMode()) {
            QMessageBox::information(this, QStringLiteral("解锁"),
                QStringLiteral("请先切换到「选择」模式，再选中要解锁的图形。"));
            return;
        }
        if (mapView_->scene()->selectedItems().isEmpty()) {
            QMessageBox::information(this, QStringLiteral("解锁"),
                QStringLiteral("请先选中图形（选择模式 → 单击图形）。"));
            return;
        }
        mapView_->setSelectedPinned(false);
    }

    void publishAreas() {
        mapView_->finalizeAllReadyShapes();
        updateProhibitionAreas();

        std::string err;
        if (!buildMapFrameProhibitionAreas(err)) {
            QMessageBox::warning(this, "Publish failed", QString::fromStdString(err));
            return;
        }
        if (prohibitionAreas_.poses.empty()) {
            QMessageBox::information(this, "No Areas",
                "No finished prohibition areas to publish.\n"
                "Polygon/Line need right-click to finish (or draw enough points then Save/Publish).");
            return;
        }
        prohibitionAreas_.header.stamp = qnode_->node()->now();
        prohibitionAreas_.header.frame_id = "map";
        qnode_->publishProhibitionAreas(prohibitionAreas_);

        const int figs = static_cast<int>(mapView_->getFinishedShapeItems().size());
        const int unfinished = mapView_->unfinishedShapeCount();
        QString msg = QString(
            "Preview only — not written to SQLite.\n"
            "Published %1 map-frame points from %2 figure(s) to prohibition_areas.\n"
            "Coordinates are local ENU meters relative to Origin Lat/Lon.\n"
            "To persist for Nav2, click Save to DB.")
            .arg(prohibitionAreas_.poses.size()).arg(figs);
        if (unfinished > 0) {
            msg += QString("\n\nSkipped %1 unfinished shape(s).").arg(unfinished);
        }
        // Show first point so user can verify non-zero meters
        if (!prohibitionAreas_.poses.empty()) {
            const auto & p = prohibitionAreas_.poses.front().position;
            msg += QString("\n\nSample map point: (%.2f, %.2f) m").arg(p.x).arg(p.y);
        }
        QMessageBox::information(this, "Published", msg);
    }

    void applyDatum() {
        bool okLat = false, okLon = false, okYaw = false;
        const double lat = originLatEdit_->text().toDouble(&okLat);
        const double lon = originLonEdit_->text().toDouble(&okLon);
        const double yawDeg = originYawEdit_->text().toDouble(&okYaw);
        if (!okLat || !okLon) {
            QMessageBox::warning(this, "Datum", "Invalid origin lat/lon.");
            return;
        }
        const double yaw = (okYaw ? yawDeg : 0.0) * M_PI / 180.0;
        qnode_->setFromLlService("/fromLL");
        qnode_->setToLlService("/toLL");
        qnode_->setDatumService("/datum");

        std::string err;
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const bool ready = qnode_->ensureNavsatTransform(err);
        QApplication::restoreOverrideCursor();
        if (!ready) {
            QMessageBox::warning(this, "navsat_transform", QString::fromStdString(err));
            return;
        }

        if (!qnode_->setDatum(lat, lon, yaw, err)) {
            QMessageBox::warning(this, "SetDatum failed", QString::fromStdString(err));
            return;
        }
        currentCenterLat_ = lat;
        currentCenterLon_ = lon;
        defaultCenterLat_ = lat;
        defaultCenterLon_ = lon;

        // Recenter smoothly on the new origin without forcing firstTileLoad_
        // (that was causing fitInView jump). Remap/reload only if needed.
        if (!isPgmMap_) {
            if (currentMinTileX_ != INT_MAX) {
                const double tileX = lon2tilex_d(lon, currentZoom_);
                const double tileY = lat2tiley_d(lat, currentZoom_);
                const QPointF sceneCenter(
                    (tileX - currentMinTileX_) * 256.0,
                    (tileY - currentMinTileY_) * 256.0);
                // If origin is far outside the current mosaic, reload tiles;
                // otherwise just center the view (no jump from fitInView).
                const QRectF sceneRect = scene_->sceneRect();
                if (!sceneRect.contains(sceneCenter)) {
                    firstTileLoad_ = false;
                    loadOnlineTileMap();
                } else {
                    mapView_->centerOn(sceneCenter);
                }
            } else {
                firstTileLoad_ = true;
                loadOnlineTileMap();
            }
        }
        QMessageBox::information(this, "Datum",
            "SetDatum applied; map recentered.\n"
            "Save/Load use Origin Lat/Lon as local ENU (0,0) — keep this consistent with the robot.");
    }

    void saveKeepoutToDb() {
        mapView_->finalizeAllReadyShapes();
        updateProhibitionAreas();

        const QString mapNameQ = currentMapName();
        if (mapNameQ.isEmpty()) {
            QMessageBox::warning(this, "Save DB",
                "MapName is required.\n"
                "Each navigation map has its own keepout set; one figure = one DB row under that MapName.");
            return;
        }

        std::string err;
        std::vector<keepout_db::Figure> figures;
        if (!buildKeepoutFigures(figures, err)) {
            QMessageBox::warning(this, "Save DB failed", QString::fromStdString(err));
            return;
        }
        if (figures.empty()) {
            QMessageBox::information(this, "Save DB",
                "No finished shapes to save.\n"
                "For Polygon/Line: left-click vertices, then right-click to finish.");
            return;
        }
        const std::string dbPath = dbPathEdit_->text().trimmed().toStdString();
        const std::string mapName = mapNameQ.toStdString();
        if (!keepout_db::replaceFigures(dbPath, mapName, figures, err)) {
            QMessageBox::warning(this, "Save DB failed", QString::fromStdString(err));
            return;
        }
        refreshMapNameList(mapNameQ);
        const int unfinished = mapView_->unfinishedShapeCount();
        QMap<QString, int> typeCount;
        for (const auto & f : figures) {
            typeCount[QString::fromStdString(f.figure_type)]++;
        }
        QStringList typeParts;
        for (auto it = typeCount.begin(); it != typeCount.end(); ++it) {
            typeParts << QString("%1×%2").arg(it.value()).arg(it.key());
        }
        QString msg;
        if (isPgmMap_) {
            msg = QString(
                "Saved %1 figure(s) for map '%2'\n"
                "Types: %3\n"
                "%4\n\n"
                "PGM map-frame meters (from YAML):\n"
                "  file=%5\n"
                "  yaml=%6\n"
                "  resolution=%7 m/px\n"
                "  origin=[%8, %9]\n"
                "  x = origin_x + px*res, y = origin_y + (H-py)*res")
                .arg(figures.size())
                .arg(mapNameQ)
                .arg(typeParts.join(", "))
                .arg(dbPathEdit_->text())
                .arg(currentPgmPath_)
                .arg(currentPgmYamlPath_)
                .arg(mapResolution_, 0, 'f', 4)
                .arg(mapOriginX_, 0, 'f', 3)
                .arg(mapOriginY_, 0, 'f', 3);
        } else {
            double lat0 = 0, lon0 = 0, yawRad = 0;
            std::string originErr;
            readOriginLatLon(lat0, lon0, yawRad, originErr);
            msg = QString(
                "Saved %1 figure(s) for map '%2'\n"
                "Types: %3\n"
                "%4\n\n"
                "vertices_json is local ENU meters relative to Origin:\n"
                "  lat=%5  lon=%6  yaw=%7°\n"
                "  x = East, y = North (must match robot navsat datum)")
                .arg(figures.size())
                .arg(mapNameQ)
                .arg(typeParts.join(", "))
                .arg(dbPathEdit_->text())
                .arg(lat0, 0, 'f', 8)
                .arg(lon0, 0, 'f', 8)
                .arg(yawRad * 180.0 / M_PI, 0, 'f', 1);
        }
        if (!figures.empty() && !figures.front().vertices.empty()) {
            const auto & p = figures.front().vertices.front();
            msg += QString("\nSample vertex: (%1, %2) m")
                .arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2);
        } else if (!figures.empty() && figures.front().figure_type == "circle") {
            msg += QString("\nSample circle center: (%1, %2) r=%3 m")
                .arg(figures.front().center_x, 0, 'f', 2)
                .arg(figures.front().center_y, 0, 'f', 2)
                .arg(figures.front().radius, 0, 'f', 2);
        }
        if (unfinished > 0) {
            msg += QString("\n\nSkipped %1 unfinished shape(s).").arg(unfinished);
        }
        QMessageBox::information(this, "Save DB", msg);
    }

    void loadKeepoutFromDb() {
        const QString mapNameQ = currentMapName();
        if (mapNameQ.isEmpty()) {
            QMessageBox::warning(this, "Load DB",
                "MapName is required.\nSelect or type the navigation map id to load its keepouts.");
            return;
        }

        std::vector<keepout_db::Figure> figures;
        std::string err;
        const std::string dbPath = dbPathEdit_->text().trimmed().toStdString();
        const std::string mapName = mapNameQ.toStdString();
        if (!keepout_db::loadFigures(dbPath, mapName, figures, err)) {
            QMessageBox::warning(this, "Load DB failed", QString::fromStdString(err));
            return;
        }
        if (figures.empty()) {
            QMessageBox::information(this, "Load DB",
                QString("No figures for map '%1'.").arg(mapNameQ));
            return;
        }
        if (isPgmMap_) {
            if (mapResolution_ <= 1e-9 || mapHeightPx_ <= 0) {
                QMessageBox::warning(this, "Load DB",
                    "PGM map metadata missing.\nReload the .pgm (with matching .yaml) first.");
                return;
            }
        } else if (currentMinTileX_ == INT_MAX) {
            QMessageBox::warning(this, "Load DB", "Load a tile map first so scene↔geo mapping exists.");
            return;
        }

        double lat0 = 0, lon0 = 0, yawRad = 0;
        if (!isPgmMap_ && !readOriginLatLon(lat0, lon0, yawRad, err)) {
            QMessageBox::warning(this, "Load DB", QString::fromStdString(err));
            return;
        }

        mapView_->clearShapes();
        int loaded = 0;
        for (const auto & f : figures) {
            if (f.figure_type == "circle") {
                QPointF cScene;
                QPointF rScene;
                if (isPgmMap_) {
                    cScene = mapMetersToScenePgm(f.center_x, f.center_y);
                    rScene = mapMetersToScenePgm(f.center_x + f.radius, f.center_y);
                } else {
                    double lat = 0, lon = 0;
                    mapMetersToLatLon(f.center_x, f.center_y, lat0, lon0, yawRad, lat, lon);
                    double lat2 = 0, lon2 = 0;
                    mapMetersToLatLon(f.center_x + f.radius, f.center_y, lat0, lon0, yawRad, lat2, lon2);
                    cScene = latLonToScene(lat, lon);
                    rScene = latLonToScene(lat2, lon2);
                }
                const double sceneR = QLineF(cScene, rScene).length();
                mapView_->addFinishedShape(ShapeItem::Circle, {}, cScene, sceneR);
                ++loaded;
            } else {
                QVector<QPointF> scenePts;
                for (const auto & p : f.vertices) {
                    if (isPgmMap_) {
                        scenePts.push_back(mapMetersToScenePgm(p.x, p.y));
                    } else {
                        double lat = 0, lon = 0;
                        mapMetersToLatLon(p.x, p.y, lat0, lon0, yawRad, lat, lon);
                        scenePts.push_back(latLonToScene(lat, lon));
                    }
                }
                ShapeItem::ShapeType t = ShapeItem::Polygon;
                if (f.figure_type == "line") {
                    t = ShapeItem::Line;
                } else if (f.figure_type == "rectangle") {
                    t = ShapeItem::Rectangle;
                } else if (f.figure_type == "polygon" && scenePts.size() == 4 &&
                           isOrthogonalQuad(scenePts)) {
                    // Legacy recovery: rectangles previously saved as polygon.
                    t = ShapeItem::Rectangle;
                    qDebug() << "Load: recovering 4-vert orthogonal polygon as Rectangle";
                }
                // Rectangle: keep all 4 corners so orientation (rotation) is restored.
                // Falling back to 2 opposite corners only for legacy 2-point rows.
                if (t == ShapeItem::Rectangle && scenePts.size() == 2) {
                    QRectF r(scenePts.first(), scenePts.last());
                    r = r.normalized();
                    scenePts = QVector<QPointF>{r.topLeft(), r.bottomRight()};
                }
                mapView_->addFinishedShape(t, scenePts);
                ++loaded;
            }
        }
        updateProhibitionAreas();
        updateInflationPreview();
        QMessageBox::information(this, "Load DB",
            QString("Loaded %1 figure(s) for map '%2'.").arg(loaded).arg(mapNameQ));
    }

    void notifyNav2CurrentMap() {
        const QString mapNameQ = currentMapName();
        if (mapNameQ.isEmpty()) {
            QMessageBox::warning(this, "Notify Nav2",
                "MapName is required.\nPublish keepout_refresh as map:<MapName> so filter_keepout loads that map.");
            return;
        }
        qnode_->publishKeepoutMapSwitch(mapNameQ.toStdString());
        QMessageBox::information(this, "Notify Nav2",
            QString("Published keepout_refresh: map:%1\n"
                    "Nav2 KeepoutFilter will load only this map's keepouts.")
                .arg(mapNameQ));
    }

    void calibrateGps() {
        applyDatum();
    }

private:
    /** Zoom of the mosaic currently painted in the scene. */
    int mosaicZoom() const {
        return (displayedTileZoom_ >= 1) ? displayedTileZoom_ : currentZoom_;
    }

    bool sceneToLatLon(const QPointF & scenePt, double & lat, double & lon) const {
        if (currentMinTileX_ == INT_MAX || mosaicZoom() < 1) {
            return false;
        }
        const int z = mosaicZoom();
        const double tileX = scenePt.x() / 256.0 + currentMinTileX_;
        const double tileY = scenePt.y() / 256.0 + currentMinTileY_;
        lon = tilex2lon_d(tileX, z);
        lat = tiley2lat_d(tileY, z);
        return true;
    }

    QPointF latLonToScene(double lat, double lon) const {
        const int z = mosaicZoom();
        const double tileX = lon2tilex_d(lon, z);
        const double tileY = lat2tiley_d(lat, z);
        return QPointF((tileX - currentMinTileX_) * 256.0, (tileY - currentMinTileY_) * 256.0);
    }

    /**
     * PGM scene pixel → map-frame meters using loaded YAML (resolution / origin).
     * Matches ROS map_server / keepout_editor: y axis flipped (image top → max map y).
     *   x = origin_x + px * resolution
     *   y = origin_y + (height - py) * resolution
     */
    bool sceneToMapMetersPgm(const QPointF & scenePt, geometry_msgs::msg::Point & out,
                             std::string & err) const {
        if (!isPgmMap_) {
            err = "Not in PGM map mode";
            return false;
        }
        if (mapResolution_ <= 1e-9) {
            err = "Invalid PGM resolution (check matching .yaml)";
            return false;
        }
        const double h = (mapHeightPx_ > 0)
            ? static_cast<double>(mapHeightPx_)
            : (mapItem_ ? static_cast<double>(mapItem_->pixmap().height()) : 0.0);
        if (h <= 0.0) {
            err = "PGM image height unknown — reload the .pgm file";
            return false;
        }
        out.x = mapOriginX_ + scenePt.x() * mapResolution_;
        out.y = mapOriginY_ + (h - scenePt.y()) * mapResolution_;
        out.z = 0.0;
        return true;
    }

    QPointF mapMetersToScenePgm(double x, double y) const {
        const double h = (mapHeightPx_ > 0)
            ? static_cast<double>(mapHeightPx_)
            : (mapItem_ ? static_cast<double>(mapItem_->pixmap().height()) : 0.0);
        return QPointF(
            (x - mapOriginX_) / mapResolution_,
            h - (y - mapOriginY_) / mapResolution_);
    }

    /** Map origin from Origin Lat/Lon fields (must match robot navsat datum). */
    bool readOriginLatLon(double & lat0, double & lon0, double & yawRad, std::string & err) const {
        bool okLat = false, okLon = false, okYaw = false;
        lat0 = originLatEdit_->text().toDouble(&okLat);
        lon0 = originLonEdit_->text().toDouble(&okLon);
        const double yawDeg = originYawEdit_->text().toDouble(&okYaw);
        if (!okLat || !okLon) {
            err = "Invalid Origin Lat/Lon (required for map-meter conversion)";
            return false;
        }
        yawRad = (okYaw ? yawDeg : 0.0) * M_PI / 180.0;
        return true;
    }

    /**
     * WGS84 local ENU meters relative to Origin (x=east, y=north), then apply yaw.
     * Used for Save/Load so we do not depend on navsat /fromLL (which returns 0,0,0
     * until /odometry/filtered arrives).
     */
    geometry_msgs::msg::Point latLonToMapMeters(double lat, double lon,
                                                double lat0, double lon0,
                                                double yawRad) const {
        constexpr double kWgs84A = 6378137.0;
        const double lat0r = lat0 * M_PI / 180.0;
        double east = (lon - lon0) * M_PI / 180.0 * kWgs84A * std::cos(lat0r);
        double north = (lat - lat0) * M_PI / 180.0 * kWgs84A;
        const double c = std::cos(yawRad);
        const double s = std::sin(yawRad);
        geometry_msgs::msg::Point p;
        p.x = c * east + s * north;
        p.y = -s * east + c * north;
        p.z = 0.0;
        return p;
    }

    void mapMetersToLatLon(double x, double y, double lat0, double lon0, double yawRad,
                           double & lat, double & lon) const {
        constexpr double kWgs84A = 6378137.0;
        const double c = std::cos(yawRad);
        const double s = std::sin(yawRad);
        const double east = c * x - s * y;
        const double north = s * x + c * y;
        const double lat0r = lat0 * M_PI / 180.0;
        lat = lat0 + (north / kWgs84A) * 180.0 / M_PI;
        lon = lon0 + (east / (kWgs84A * std::cos(lat0r))) * 180.0 / M_PI;
    }

    bool buildKeepoutFigures(std::vector<keepout_db::Figure> & figures, std::string & err) {
        figures.clear();
        double lat0 = 0, lon0 = 0, yawRad = 0;
        if (!isPgmMap_) {
            if (!readOriginLatLon(lat0, lon0, yawRad, err)) {
                return false;
            }
        } else if (mapResolution_ <= 1e-9) {
            err = "PGM resolution invalid — reload .pgm with matching .yaml "
                  "(resolution / origin)";
            return false;
        }

        auto shapes = mapView_->getFinishedShapeItems();
        int idx = 0;
        for (ShapeItem * shape : shapes) {
            keepout_db::Figure fig;
            fig.map_name = currentMapName().toStdString();
            if (fig.map_name.empty()) {
                err = "MapName is required before saving keepouts";
                return false;
            }
            fig.figure_name = "fig_" + std::to_string(++idx);

            if (shape->shapeType() == ShapeItem::Circle) {
                fig.figure_type = "circle";
                geometry_msgs::msg::Point centerMap;
                geometry_msgs::msg::Point rimMap;
                if (isPgmMap_) {
                    if (!sceneToMapMetersPgm(shape->sceneCenter(), centerMap, err)) {
                        return false;
                    }
                    QPointF rimScene = shape->sceneCenter() + QPointF(shape->sceneRadius(), 0.0);
                    if (!sceneToMapMetersPgm(rimScene, rimMap, err)) {
                        return false;
                    }
                } else {
                    double lat = 0, lon = 0;
                    if (!sceneToLatLon(shape->sceneCenter(), lat, lon)) {
                        err = "Cannot convert circle center (load tile map first)";
                        return false;
                    }
                    centerMap = latLonToMapMeters(lat, lon, lat0, lon0, yawRad);
                    QPointF rimScene = shape->sceneCenter() + QPointF(shape->sceneRadius(), 0.0);
                    double lat2 = 0, lon2 = 0;
                    if (!sceneToLatLon(rimScene, lat2, lon2)) {
                        err = "Cannot convert circle rim";
                        return false;
                    }
                    rimMap = latLonToMapMeters(lat2, lon2, lat0, lon0, yawRad);
                }
                fig.center_x = centerMap.x;
                fig.center_y = centerMap.y;
                fig.radius = std::hypot(rimMap.x - centerMap.x, rimMap.y - centerMap.y);
                if (fig.radius < 1e-3) {
                    err = isPgmMap_
                        ? "Circle radius converted to ~0 m — check PGM YAML resolution"
                        : "Circle radius converted to ~0 m — check Origin Lat/Lon and tile map";
                    return false;
                }
                qDebug() << "keepout save:" << fig.figure_name.c_str()
                         << "type" << fig.figure_type.c_str()
                         << "enum" << static_cast<int>(shape->shapeType());
                figures.push_back(fig);
                continue;
            }

            // Prefer the type stamped at construction (avoids any enum mismatch).
            fig.figure_type = shape->dbFigureType();
            if (fig.figure_type.empty()) {
                switch (shape->shapeType()) {
                    case ShapeItem::Line: fig.figure_type = "line"; break;
                    case ShapeItem::Rectangle: fig.figure_type = "rectangle"; break;
                    case ShapeItem::Polygon: fig.figure_type = "polygon"; break;
                    default: fig.figure_type = "polygon"; break;
                }
            }

            const auto verts = shape->sceneVertices();
            qDebug() << "keepout save:" << fig.figure_name.c_str()
                     << "type" << fig.figure_type.c_str()
                     << "enum" << static_cast<int>(shape->shapeType())
                     << "verts" << verts.size();
            if (fig.figure_type == "line" && verts.size() < 2) {
                continue;
            }
            if ((fig.figure_type == "polygon" || fig.figure_type == "rectangle") &&
                verts.size() < 3) {
                continue;
            }
            double minX = 1e100, maxX = -1e100, minY = 1e100, maxY = -1e100;
            for (const auto & sp : verts) {
                geometry_msgs::msg::Point mp;
                if (isPgmMap_) {
                    if (!sceneToMapMetersPgm(sp, mp, err)) {
                        return false;
                    }
                } else {
                    double lat = 0, lon = 0;
                    if (!sceneToLatLon(sp, lat, lon)) {
                        err = "Cannot convert vertex (load tile map first)";
                        return false;
                    }
                    mp = latLonToMapMeters(lat, lon, lat0, lon0, yawRad);
                }
                minX = std::min(minX, mp.x);
                maxX = std::max(maxX, mp.x);
                minY = std::min(minY, mp.y);
                maxY = std::max(maxY, mp.y);
                fig.vertices.push_back(mp);
            }
            // Guard against the old fromLL-all-zeros failure mode.
            if ((maxX - minX) < 1e-3 && (maxY - minY) < 1e-3) {
                err = isPgmMap_
                    ? "All vertices collapsed to nearly the same map point (~0 span). "
                      "Check PGM YAML resolution/origin."
                    : "All vertices collapsed to nearly the same map point (~0 span). "
                      "Check Origin Lat/Lon matches the drawn area.";
                return false;
            }
            figures.push_back(fig);
        }
        return true;
    }

    bool buildMapFrameProhibitionAreas(std::string & err) {
        prohibitionAreas_.poses.clear();
        std::vector<keepout_db::Figure> figures;
        if (!buildKeepoutFigures(figures, err)) {
            return false;
        }
        for (const auto & f : figures) {
            if (f.figure_type == "circle") {
                // sample circle for PoseArray preview
                const int N = 16;
                for (int i = 0; i < N; ++i) {
                    const double a = 2.0 * M_PI * i / N;
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = f.center_x + f.radius * std::cos(a);
                    pose.position.y = f.center_y + f.radius * std::sin(a);
                    prohibitionAreas_.poses.push_back(pose);
                }
            } else {
                for (const auto & p : f.vertices) {
                    geometry_msgs::msg::Pose pose;
                    pose.position = p;
                    prohibitionAreas_.poses.push_back(pose);
                }
            }
        }
        return true;
    }

    static double lon2tilex_d(double lon, int z) {
        return (lon + 180.0) / 360.0 * (1 << z);
    }
    static double lat2tiley_d(double lat, int z) {
        const double latrad = lat * M_PI / 180.0;
        return (1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z);
    }
    static double tilex2lon_d(double x, int z) {
        return x / static_cast<double>(1 << z) * 360.0 - 180.0;
    }
    static double tiley2lat_d(double y, int z) {
        const double n = M_PI - (y / static_cast<double>(1 << z)) * 2.0 * M_PI;
        return atan(sinh(n)) * 180.0 / M_PI;
    }

private:
    QWidget * makeDrawerPage(const QString & titleHint = QString()) {
        Q_UNUSED(titleHint);
        QWidget * page = new QWidget();
        QVBoxLayout * lay = new QVBoxLayout(page);
        lay->setContentsMargins(8, 8, 8, 8);
        lay->setSpacing(8);
        return page;
    }

    static QToolButton * makeModeToolButton(const QString & text, const QString & tip) {
        QToolButton * btn = new QToolButton();
        btn->setText(text);
        btn->setToolTip(tip);
        btn->setCheckable(true);
        btn->setToolButtonStyle(Qt::ToolButtonTextOnly);
        btn->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        btn->setMinimumHeight(32);
        return btn;
    }

    void setupUI() {
        setWindowTitle(QStringLiteral("禁行区编辑 — 地图绘制"));
        resize(1400, 860);
        setMinimumSize(960, 640);

        setStyleSheet(QStringLiteral(
            "QToolBox::tab { background: #eceff1; padding: 8px 10px; font-weight: 600; }"
            "QToolBox::tab:selected { background: #cfd8dc; color: #0d47a1; }"
            "QFrame#SidebarFrame { background: #f5f7fa; border-right: 1px solid #cfd8dc; }"
            "QFrame#MapChrome { background: #263238; }"
            "QToolButton:checked { background: #1565c0; color: white; border-radius: 4px; }"
            "QToolButton { padding: 4px 8px; border: 1px solid #90a4ae; border-radius: 4px; background: #eceff1; }"
            "QStatusBar, QLabel#StatusHint { color: #37474f; }"
            "QPushButton { padding: 6px 10px; }"
        ));

        QVBoxLayout * root = new QVBoxLayout(this);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(0);

        QSplitter * splitter = new QSplitter(Qt::Horizontal, this);
        splitter->setHandleWidth(4);
        splitter->setChildrenCollapsible(false);

        // ── Left drawer (一级 QToolBox / 二级控件) ──
        sidebarFrame_ = new QFrame();
        sidebarFrame_->setObjectName(QStringLiteral("SidebarFrame"));
        sidebarFrame_->setMinimumWidth(280);
        sidebarFrame_->setMaximumWidth(420);
        QVBoxLayout * sideLay = new QVBoxLayout(sidebarFrame_);
        sideLay->setContentsMargins(0, 0, 0, 0);
        sideLay->setSpacing(0);

        QLabel * brand = new QLabel(QStringLiteral("  禁行区编辑"));
        brand->setStyleSheet(QStringLiteral(
            "font-size: 15px; font-weight: 700; padding: 12px 10px;"
            "background: #1565c0; color: white;"));
        sideLay->addWidget(brand);

        QToolBox * toolbox = new QToolBox();
        toolbox->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

        // —— 一级：地图 ——
        QWidget * mapPage = makeDrawerPage();
        QVBoxLayout * mapLay = qobject_cast<QVBoxLayout *>(mapPage->layout());
        mapLay->addWidget(new QLabel(QStringLiteral("地图类型")));
        mapTypeCombo_ = new QComboBox();
        mapTypeCombo_->addItem(QStringLiteral("在线瓦片地图"), QStringLiteral("Online Tile Map"));
        mapTypeCombo_->addItem(QStringLiteral("本地 PGM 地图"), QStringLiteral("Local PGM Map"));
        mapTypeCombo_->setCurrentIndex(0);
        mapLay->addWidget(mapTypeCombo_);

        mapLay->addWidget(new QLabel(QStringLiteral("瓦片 URL")));
        tileUrlEdit_ = new QLineEdit(defaultTileUrl_);
        tileUrlEdit_->setPlaceholderText(QStringLiteral("WMTS / XYZ …"));
        mapLay->addWidget(tileUrlEdit_);

        loadMapBtn_ = new QPushButton(QStringLiteral("加载地图"));
        mapLay->addWidget(loadMapBtn_);

        QHBoxLayout * zoomRow = new QHBoxLayout();
        zoomInBtn_ = new QPushButton(QStringLiteral("放大"));
        zoomOutBtn_ = new QPushButton(QStringLiteral("缩小"));
        resetZoomBtn_ = new QPushButton(QStringLiteral("复位"));
        zoomRow->addWidget(zoomInBtn_);
        zoomRow->addWidget(zoomOutBtn_);
        zoomRow->addWidget(resetZoomBtn_);
        mapLay->addLayout(zoomRow);
        mapLay->addStretch(1);
        toolbox->addItem(mapPage, QStringLiteral("① 地图"));

        // —— 一级：禁行绘制 ——
        QWidget * drawPage = makeDrawerPage();
        QVBoxLayout * drawLay = qobject_cast<QVBoxLayout *>(drawPage->layout());

        drawLay->addWidget(new QLabel(QStringLiteral("绘制工具（选中即进入对应模式）")));
        QGridLayout * toolGrid = new QGridLayout();
        toolGrid->setSpacing(6);
        selectToolBtn_ = makeModeToolButton(QStringLiteral("选择"),
            QStringLiteral("编辑/平移：选中、拖动、删除已有禁行区"));
        lineToolBtn_ = makeModeToolButton(QStringLiteral("线段"),
            QStringLiteral("绘制禁行线"));
        rectToolBtn_ = makeModeToolButton(QStringLiteral("矩形"),
            QStringLiteral("拖拽绘制矩形禁行区"));
        circleToolBtn_ = makeModeToolButton(QStringLiteral("圆形"),
            QStringLiteral("拖拽绘制圆形禁行区"));
        polygonToolBtn_ = makeModeToolButton(QStringLiteral("多边形"),
            QStringLiteral("逐点绘制多边形，右键结束"));
        lineToolBtn_->setChecked(true);

        drawToolGroup_ = new QButtonGroup(this);
        drawToolGroup_->setExclusive(true);
        drawToolGroup_->addButton(selectToolBtn_);
        drawToolGroup_->addButton(lineToolBtn_);
        drawToolGroup_->addButton(rectToolBtn_);
        drawToolGroup_->addButton(circleToolBtn_);
        drawToolGroup_->addButton(polygonToolBtn_);

        toolGrid->addWidget(selectToolBtn_, 0, 0);
        toolGrid->addWidget(lineToolBtn_, 0, 1);
        toolGrid->addWidget(rectToolBtn_, 1, 0);
        toolGrid->addWidget(circleToolBtn_, 1, 1);
        toolGrid->addWidget(polygonToolBtn_, 2, 0, 1, 2);
        drawLay->addLayout(toolGrid);

        // Keep legacy radio pointers null / unused — selectedShapeTool uses tool buttons
        lineRadio_ = nullptr;
        rectRadio_ = nullptr;
        circleRadio_ = nullptr;
        polygonRadio_ = nullptr;

        drawLay->addWidget(new QLabel(QStringLiteral("编辑操作")));
        QGridLayout * editGrid = new QGridLayout();
        undoDrawBtn_ = new QPushButton(QStringLiteral("撤销"));
        undoDrawBtn_->setToolTip(QStringLiteral("Ctrl+Z"));
        deleteSelectedBtn_ = new QPushButton(QStringLiteral("删除"));
        deleteSelectedBtn_->setToolTip(QStringLiteral("Delete / Backspace"));
        pinBtn_ = new QPushButton(QStringLiteral("钉住"));
        unpinBtn_ = new QPushButton(QStringLiteral("解锁"));
        clearShapesBtn_ = new QPushButton(QStringLiteral("清空全部"));
        editGrid->addWidget(undoDrawBtn_, 0, 0);
        editGrid->addWidget(deleteSelectedBtn_, 0, 1);
        editGrid->addWidget(pinBtn_, 1, 0);
        editGrid->addWidget(unpinBtn_, 1, 1);
        editGrid->addWidget(clearShapesBtn_, 2, 0, 1, 2);
        drawLay->addLayout(editGrid);

        // Nav2 keepout_filter inflation debug preview
        QGroupBox * inflBox = new QGroupBox(QStringLiteral("Nav2 膨胀预览 (调试)"));
        QFormLayout * inflForm = new QFormLayout(inflBox);
        inflForm->setContentsMargins(6, 8, 6, 6);
        inflForm->setSpacing(6);
        showInflationChk_ = new QCheckBox(QStringLiteral("在地图上显示膨胀范围"));
        showInflationChk_->setChecked(true);
        showInflationChk_->setToolTip(
            QStringLiteral("对应 filter_keepout：橙色=inscribed_radius，青色=inflation_radius"));
        inflForm->addRow(showInflationChk_);
        inscribedRadiusSpin_ = new QDoubleSpinBox();
        inscribedRadiusSpin_->setRange(0.0, 5.0);
        inscribedRadiusSpin_->setSingleStep(0.05);
        inscribedRadiusSpin_->setDecimals(2);
        inscribedRadiusSpin_->setSuffix(QStringLiteral(" m"));
        inscribedRadiusSpin_->setValue(0.40);
        inscribedRadiusSpin_->setToolTip(
            QStringLiteral("keepout_filter.inscribed_radius — 内切半径"));
        inflationRadiusSpin_ = new QDoubleSpinBox();
        inflationRadiusSpin_->setRange(0.0, 10.0);
        inflationRadiusSpin_->setSingleStep(0.05);
        inflationRadiusSpin_->setDecimals(2);
        inflationRadiusSpin_->setSuffix(QStringLiteral(" m"));
        inflationRadiusSpin_->setValue(1.0);
        inflationRadiusSpin_->setToolTip(
            QStringLiteral("keepout_filter.inflation_radius — 建议 ≥ footprint 外接半径 (~0.94 m)"));
        inflForm->addRow(QStringLiteral("inscribed_radius"), inscribedRadiusSpin_);
        inflForm->addRow(QStringLiteral("inflation_radius"), inflationRadiusSpin_);
        QLabel * inflHint = new QLabel(
            QStringLiteral("仅 GUI 预览，不写库。调好后写入 nav2 keepout_filter。\n"
                           "橙带 = inscribed，青带 = inflation。"));
        inflHint->setWordWrap(true);
        inflHint->setStyleSheet(QStringLiteral("color:#546e7a; font-size:11px;"));
        inflForm->addRow(inflHint);
        drawLay->addWidget(inflBox);

        // Hidden compatibility: Start/Stop map to tool selection
        startDrawBtn_ = new QPushButton(QStringLiteral("开始绘制"));
        stopDrawBtn_ = new QPushButton(QStringLiteral("停止绘制"));
        startDrawBtn_->setVisible(false);
        stopDrawBtn_->setVisible(false);

        QLabel * drawHint = new QLabel(
            QStringLiteral("线段/矩形/圆：按住拖拽；多边形：左键加点，右键结束。\n"
                           "切换到「选择」可编辑已有图形。"));
        drawHint->setWordWrap(true);
        drawHint->setStyleSheet(QStringLiteral("color:#546e7a; font-size:11px;"));
        drawLay->addWidget(drawHint);
        drawLay->addStretch(1);
        toolbox->addItem(drawPage, QStringLiteral("② 禁行绘制"));

        // —— 一级：坐标与数据 ——
        QWidget * dataPage = makeDrawerPage();
        QVBoxLayout * dataLay = qobject_cast<QVBoxLayout *>(dataPage->layout());
        QScrollArea * dataScroll = new QScrollArea();
        dataScroll->setWidgetResizable(true);
        dataScroll->setFrameShape(QFrame::NoFrame);
        QWidget * dataInner = new QWidget();
        QVBoxLayout * dataInnerLay = new QVBoxLayout(dataInner);
        dataInnerLay->setContentsMargins(4, 4, 4, 4);
        dataInnerLay->setSpacing(8);

        coordModeStack_ = new QStackedWidget();

        // Tile / GPS origin panel
        QWidget * tileCoordPage = new QWidget();
        QFormLayout * tileForm = new QFormLayout(tileCoordPage);
        tileForm->setLabelAlignment(Qt::AlignRight);
        tileForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        tileForm->setContentsMargins(0, 0, 0, 0);
        tileForm->setSpacing(6);
        gpsLatEdit_ = new QLineEdit("0.0");
        gpsLonEdit_ = new QLineEdit("0.0");
        gpsLatEdit_->setReadOnly(true);
        gpsLonEdit_->setReadOnly(true);
        tileForm->addRow(QStringLiteral("GPS Lat"), gpsLatEdit_);
        tileForm->addRow(QStringLiteral("GPS Lon"), gpsLonEdit_);
        originLatEdit_ = new QLineEdit(QString::number(defaultCenterLat_, 'f', 8));
        originLonEdit_ = new QLineEdit(QString::number(defaultCenterLon_, 'f', 8));
        originYawEdit_ = new QLineEdit("0.0");
        originLatEdit_->setToolTip(QStringLiteral("地图原点纬度，须与机器人 navsat datum 一致"));
        originLonEdit_->setToolTip(QStringLiteral("地图原点经度，须与机器人 navsat datum 一致"));
        tileForm->addRow(QStringLiteral("原点 Lat"), originLatEdit_);
        tileForm->addRow(QStringLiteral("原点 Lon"), originLonEdit_);
        tileForm->addRow(QStringLiteral("原点 Yaw°"), originYawEdit_);
        QLabel * tileHint = new QLabel(
            QStringLiteral("瓦片模式：场景→经纬度→相对 Origin 的 ENU 米制。"));
        tileHint->setWordWrap(true);
        tileHint->setStyleSheet(QStringLiteral("color:#546e7a; font-size:11px;"));
        tileForm->addRow(tileHint);
        coordModeStack_->addWidget(tileCoordPage);

        // PGM YAML metadata panel
        QWidget * pgmCoordPage = new QWidget();
        QFormLayout * pgmForm = new QFormLayout(pgmCoordPage);
        pgmForm->setLabelAlignment(Qt::AlignRight);
        pgmForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        pgmForm->setContentsMargins(0, 0, 0, 0);
        pgmForm->setSpacing(6);
        pgmYamlPathEdit_ = new QLineEdit();
        pgmImageEdit_ = new QLineEdit();
        pgmModeEdit_ = new QLineEdit();
        pgmResolutionEdit_ = new QLineEdit();
        pgmOriginXEdit_ = new QLineEdit();
        pgmOriginYEdit_ = new QLineEdit();
        pgmOriginYawEdit_ = new QLineEdit();
        pgmNegateEdit_ = new QLineEdit();
        pgmOccupiedEdit_ = new QLineEdit();
        pgmFreeEdit_ = new QLineEdit();
        pgmSizeEdit_ = new QLineEdit();
        for (QLineEdit * ed : {pgmYamlPathEdit_, pgmImageEdit_, pgmModeEdit_, pgmResolutionEdit_,
                               pgmOriginXEdit_, pgmOriginYEdit_, pgmOriginYawEdit_,
                               pgmNegateEdit_, pgmOccupiedEdit_, pgmFreeEdit_, pgmSizeEdit_}) {
            ed->setReadOnly(true);
        }
        pgmForm->addRow(QStringLiteral("YAML"), pgmYamlPathEdit_);
        pgmForm->addRow(QStringLiteral("image"), pgmImageEdit_);
        pgmForm->addRow(QStringLiteral("mode"), pgmModeEdit_);
        pgmForm->addRow(QStringLiteral("resolution"), pgmResolutionEdit_);
        pgmForm->addRow(QStringLiteral("origin X"), pgmOriginXEdit_);
        pgmForm->addRow(QStringLiteral("origin Y"), pgmOriginYEdit_);
        pgmForm->addRow(QStringLiteral("origin yaw"), pgmOriginYawEdit_);
        pgmForm->addRow(QStringLiteral("negate"), pgmNegateEdit_);
        pgmForm->addRow(QStringLiteral("occupied_thresh"), pgmOccupiedEdit_);
        pgmForm->addRow(QStringLiteral("free_thresh"), pgmFreeEdit_);
        pgmForm->addRow(QStringLiteral("图像尺寸"), pgmSizeEdit_);
        QLabel * pgmHint = new QLabel(
            QStringLiteral("PGM 模式：用同名 YAML 的 resolution/origin 转到 map 米制，不用 Origin Lat/Lon。"));
        pgmHint->setWordWrap(true);
        pgmHint->setStyleSheet(QStringLiteral("color:#546e7a; font-size:11px;"));
        pgmForm->addRow(pgmHint);
        coordModeStack_->addWidget(pgmCoordPage);

        dataInnerLay->addWidget(coordModeStack_);

        QFormLayout * sharedForm = new QFormLayout();
        sharedForm->setLabelAlignment(Qt::AlignRight);
        sharedForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        sharedForm->setContentsMargins(0, 0, 0, 0);
        sharedForm->setSpacing(6);
        dbPathEdit_ = new QLineEdit(QDir::homePath() + "/gps_filter_ws/data/keepout.db");
        sharedForm->addRow(QStringLiteral("数据库"), dbPathEdit_);

        QWidget * mapNameRow = new QWidget();
        QHBoxLayout * mapNameLay = new QHBoxLayout(mapNameRow);
        mapNameLay->setContentsMargins(0, 0, 0, 0);
        mapNameCombo_ = new QComboBox();
        mapNameCombo_->setEditable(true);
        mapNameCombo_->setInsertPolicy(QComboBox::NoInsert);
        mapNameCombo_->lineEdit()->setPlaceholderText(QStringLiteral("如 warehouse_a"));
        refreshMapsBtn_ = new QPushButton(QStringLiteral("↻"));
        refreshMapsBtn_->setToolTip(QStringLiteral("从数据库刷新地图列表"));
        refreshMapsBtn_->setFixedWidth(32);
        mapNameLay->addWidget(mapNameCombo_);
        mapNameLay->addWidget(refreshMapsBtn_);
        sharedForm->addRow(QStringLiteral("MapName"), mapNameRow);
        dataInnerLay->addLayout(sharedForm);
        dataInnerLay->addStretch(1);

        dataScroll->setWidget(dataInner);
        dataLay->addWidget(dataScroll, 1);

        applyDatumBtn_ = new QPushButton(QStringLiteral("应用 Datum"));
        saveDbBtn_ = new QPushButton(QStringLiteral("保存到数据库"));
        loadDbBtn_ = new QPushButton(QStringLiteral("从数据库加载"));
        notifyNav2Btn_ = new QPushButton(QStringLiteral("通知 Nav2 切换地图"));
        calibrateBtn_ = new QPushButton(QStringLiteral("标定 GPS (=应用 Datum)"));
        dataLay->addWidget(applyDatumBtn_);
        dataLay->addWidget(saveDbBtn_);
        dataLay->addWidget(loadDbBtn_);
        dataLay->addWidget(notifyNav2Btn_);
        dataLay->addWidget(calibrateBtn_);
        toolbox->addItem(dataPage, QStringLiteral("③ 坐标与数据"));
        refreshCoordPanelForMapMode();

        // —— 一级：发布 ——
        QWidget * pubPage = makeDrawerPage();
        QVBoxLayout * pubLay = qobject_cast<QVBoxLayout *>(pubPage->layout());
        publishBtn_ = new QPushButton(QStringLiteral("发布预览（仅话题，不写库）"));
        pubLay->addWidget(publishBtn_);
        QLabel * pubHint = new QLabel(
            QStringLiteral("预览会把当前禁行区转到 map 坐标系并发布到 prohibition_areas。\n"
                           "持久化请用「保存到数据库」。"));
        pubHint->setWordWrap(true);
        pubHint->setStyleSheet(QStringLiteral("color:#546e7a; font-size:11px;"));
        pubLay->addWidget(pubHint);
        pubLay->addStretch(1);
        toolbox->addItem(pubPage, QStringLiteral("④ 发布"));

        // —— 一级：PGM 辅助（map_editor 移植）——
        if (pgmAssist_) {
            QWidget * pgmPage = makeDrawerPage();
            QVBoxLayout * pgmLay = qobject_cast<QVBoxLayout *>(pgmPage->layout());
            QScrollArea * pgmScroll = new QScrollArea();
            pgmScroll->setWidgetResizable(true);
            pgmScroll->setFrameShape(QFrame::NoFrame);
            pgmScroll->setWidget(pgmAssist_->createPanel());
            pgmLay->addWidget(pgmScroll, 1);
            toolbox->addItem(pgmPage, QStringLiteral("⑤ 路径绘制"));
        }

        toolbox->setCurrentIndex(1);  // default open 禁行绘制
        sideLay->addWidget(toolbox, 1);
        splitter->addWidget(sidebarFrame_);

        // ── Main: map + overlay chrome ──
        QWidget * mapHost = new QWidget();
        QVBoxLayout * mapHostLay = new QVBoxLayout(mapHost);
        mapHostLay->setContentsMargins(0, 0, 0, 0);
        mapHostLay->setSpacing(0);

        QFrame * chrome = new QFrame();
        chrome->setObjectName(QStringLiteral("MapChrome"));
        QHBoxLayout * chromeLay = new QHBoxLayout(chrome);
        chromeLay->setContentsMargins(8, 6, 8, 6);
        chromeLay->setSpacing(8);

        drawerToggleBtn_ = new QPushButton(QStringLiteral("◀ 收起"));
        drawerToggleBtn_->setFixedHeight(28);
        drawerToggleBtn_->setStyleSheet(QStringLiteral(
            "QPushButton { background:#455a64; color:white; border:none; border-radius:4px; padding:4px 10px; }"
            "QPushButton:hover { background:#546e7a; }"));
        chromeLay->addWidget(drawerToggleBtn_);

        QLabel * chromeTitle = new QLabel(QStringLiteral("地图视图"));
        chromeTitle->setStyleSheet(QStringLiteral("color:#eceff1; font-weight:600;"));
        chromeLay->addWidget(chromeTitle);
        chromeLay->addStretch(1);

        // Map-top quick tools (mirrored with sidebar tool buttons)
        chromeSelectBtn_ = makeModeToolButton(QStringLiteral("选择"), selectToolBtn_->toolTip());
        chromeLineBtn_ = makeModeToolButton(QStringLiteral("线"), lineToolBtn_->toolTip());
        chromeRectBtn_ = makeModeToolButton(QStringLiteral("矩"), rectToolBtn_->toolTip());
        chromeCircleBtn_ = makeModeToolButton(QStringLiteral("圆"), circleToolBtn_->toolTip());
        chromePolyBtn_ = makeModeToolButton(QStringLiteral("多"), polygonToolBtn_->toolTip());
        for (QToolButton * b : {chromeSelectBtn_, chromeLineBtn_, chromeRectBtn_,
                                chromeCircleBtn_, chromePolyBtn_}) {
            b->setFixedHeight(28);
            b->setMinimumWidth(40);
            b->setStyleSheet(QStringLiteral(
                "QToolButton { background:#37474f; color:#eceff1; border:1px solid #546e7a; }"
                "QToolButton:checked { background:#1565c0; color:white; border-color:#42a5f5; }"));
            chromeLay->addWidget(b);
        }

        auto mirrorPair = [this](QToolButton * side, QToolButton * chrome) {
            connect(side, &QToolButton::toggled, chrome, [chrome](bool on) {
                QSignalBlocker b(chrome);
                chrome->setChecked(on);
            });
            connect(chrome, &QToolButton::clicked, this, [side]() {
                side->setChecked(true);
            });
        };
        mirrorPair(selectToolBtn_, chromeSelectBtn_);
        mirrorPair(lineToolBtn_, chromeLineBtn_);
        mirrorPair(rectToolBtn_, chromeRectBtn_);
        mirrorPair(circleToolBtn_, chromeCircleBtn_);
        mirrorPair(polygonToolBtn_, chromePolyBtn_);
        chromeLineBtn_->setChecked(true);

        mapHostLay->addWidget(chrome);

        scene_ = new QGraphicsScene(this);
        mapItem_ = new QGraphicsPixmapItem();
        scene_->addItem(mapItem_);
        mapView_ = new MapGraphicsView(scene_, this);
        mapView_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        mapView_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        mapView_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        mapHostLay->addWidget(mapView_, 1);

        splitter->addWidget(mapHost);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);
        splitter->setSizes({320, 1080});

        root->addWidget(splitter, 1);

        // Status bar
        QFrame * status = new QFrame();
        status->setStyleSheet(QStringLiteral("background:#eceff1; border-top:1px solid #cfd8dc;"));
        QHBoxLayout * statusLay = new QHBoxLayout(status);
        statusLay->setContentsMargins(10, 4, 10, 4);
        drawingToolLabel_ = new QLabel();
        drawingToolLabel_->setObjectName(QStringLiteral("StatusHint"));
        drawingToolLabel_->setWordWrap(false);
        statusLay->addWidget(drawingToolLabel_, 1);
        root->addWidget(status);

        updateDrawingToolLabel();
    }

    void setupConnections() {
        connect(loadMapBtn_, &QPushButton::clicked, this, &MainWindowWrapper::loadMap);
        connect(zoomInBtn_, &QPushButton::clicked, this, &MainWindowWrapper::zoomIn);
        connect(zoomOutBtn_, &QPushButton::clicked, this, &MainWindowWrapper::zoomOut);
        connect(resetZoomBtn_, &QPushButton::clicked, this, &MainWindowWrapper::resetZoom);
        connect(startDrawBtn_, &QPushButton::clicked, this, &MainWindowWrapper::startDrawing);
        connect(stopDrawBtn_, &QPushButton::clicked, this, &MainWindowWrapper::stopDrawing);
        connect(undoDrawBtn_, &QPushButton::clicked, this, &MainWindowWrapper::undoDrawing);
        connect(clearShapesBtn_, &QPushButton::clicked, this, &MainWindowWrapper::clearShapes);
        connect(pinBtn_, &QPushButton::clicked, this, &MainWindowWrapper::pinSelectedShapes);
        connect(unpinBtn_, &QPushButton::clicked, this, &MainWindowWrapper::unpinSelectedShapes);
        connect(deleteSelectedBtn_, &QPushButton::clicked, this, &MainWindowWrapper::deleteSelectedShapes);
        connect(drawerToggleBtn_, &QPushButton::clicked, this, &MainWindowWrapper::toggleSidebar);

        auto connectTool = [this](QToolButton * btn) {
            connect(btn, &QToolButton::toggled, this, [this](bool on) {
                if (on) onShapeToolChanged();
            });
        };
        connectTool(selectToolBtn_);
        connectTool(lineToolBtn_);
        connectTool(rectToolBtn_);
        connectTool(circleToolBtn_);
        connectTool(polygonToolBtn_);

        connect(publishBtn_, &QPushButton::clicked, this, &MainWindowWrapper::publishAreas);
        connect(calibrateBtn_, &QPushButton::clicked, this, &MainWindowWrapper::calibrateGps);
        connect(applyDatumBtn_, &QPushButton::clicked, this, &MainWindowWrapper::applyDatum);
        connect(saveDbBtn_, &QPushButton::clicked, this, &MainWindowWrapper::saveKeepoutToDb);
        connect(loadDbBtn_, &QPushButton::clicked, this, &MainWindowWrapper::loadKeepoutFromDb);
        connect(notifyNav2Btn_, &QPushButton::clicked, this, &MainWindowWrapper::notifyNav2CurrentMap);
        connect(refreshMapsBtn_, &QPushButton::clicked, this, [this]() { refreshMapNameList(); });

        connect(mapView_, &MapGraphicsView::shapeFinished, this, &MainWindowWrapper::onShapeFinished);
        connect(mapView_, &MapGraphicsView::shapeEdited, this, &MainWindowWrapper::onShapeFinished);
        connect(mapView_, &MapGraphicsView::zoomDelta, this, &MainWindowWrapper::onZoomChanged);
        connect(qnode_, &QNode::gpsUpdated, this, &MainWindowWrapper::onGpsUpdate);
        connect(mapView_, &MapGraphicsView::panFinished, this, &MainWindowWrapper::onViewChanged);

        connect(showInflationChk_, &QCheckBox::toggled, this, [this](bool) { updateInflationPreview(); });
        connect(inscribedRadiusSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this](double) { updateInflationPreview(); });
        connect(inflationRadiusSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this](double) { updateInflationPreview(); });

        // Default: enter line drawing when UI is ready
        QTimer::singleShot(0, this, [this]() {
            if (lineToolBtn_) {
                lineToolBtn_->setChecked(true);
                onShapeToolChanged();
            }
            updateInflationPreview();
        });
    }

    void updateProhibitionAreas() {
        // Lightweight scene-coord preview; map-frame conversion on Publish/Save.
        prohibitionAreas_.poses.clear();
        for (ShapeItem * shape : mapView_->getFinishedShapeItems()) {
            if (shape->shapeType() == ShapeItem::Circle) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = shape->sceneCenter().x();
                pose.position.y = shape->sceneCenter().y();
                prohibitionAreas_.poses.push_back(pose);
            } else {
                for (const auto & sp : shape->sceneVertices()) {
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = sp.x();
                    pose.position.y = sp.y();
                    prohibitionAreas_.poses.push_back(pose);
                }
            }
        }
    }

    /** Switch「坐标与数据」between GPS Origin (tile) and PGM YAML metadata. */
    void refreshCoordPanelForMapMode() {
        if (!coordModeStack_) {
            return;
        }
        if (isPgmMap_) {
            coordModeStack_->setCurrentIndex(1);
            if (pgmYamlPathEdit_) {
                pgmYamlPathEdit_->setText(currentPgmYamlPath_);
            }
            if (pgmImageEdit_) {
                pgmImageEdit_->setText(pgmYamlImage_.isEmpty()
                    ? QFileInfo(currentPgmPath_).fileName()
                    : pgmYamlImage_);
            }
            if (pgmModeEdit_) {
                pgmModeEdit_->setText(pgmYamlMode_);
            }
            if (pgmResolutionEdit_) {
                pgmResolutionEdit_->setText(QString::number(mapResolution_, 'g', 10));
            }
            if (pgmOriginXEdit_) {
                pgmOriginXEdit_->setText(QString::number(mapOriginX_, 'g', 10));
            }
            if (pgmOriginYEdit_) {
                pgmOriginYEdit_->setText(QString::number(mapOriginY_, 'g', 10));
            }
            if (pgmOriginYawEdit_) {
                pgmOriginYawEdit_->setText(QString::number(mapOriginYaw_, 'g', 10));
            }
            if (pgmNegateEdit_) {
                pgmNegateEdit_->setText(QString::number(pgmYamlNegate_));
            }
            if (pgmOccupiedEdit_) {
                pgmOccupiedEdit_->setText(QString::number(pgmOccupiedThresh_, 'g', 10));
            }
            if (pgmFreeEdit_) {
                pgmFreeEdit_->setText(QString::number(pgmFreeThresh_, 'g', 10));
            }
            if (pgmSizeEdit_) {
                pgmSizeEdit_->setText(
                    QStringLiteral("%1 × %2 px").arg(mapWidthPx_).arg(mapHeightPx_));
            }
            if (applyDatumBtn_) {
                applyDatumBtn_->setVisible(false);
            }
            if (calibrateBtn_) {
                calibrateBtn_->setVisible(false);
            }
        } else {
            coordModeStack_->setCurrentIndex(0);
            if (applyDatumBtn_) {
                applyDatumBtn_->setVisible(true);
            }
            if (calibrateBtn_) {
                calibrateBtn_->setVisible(true);
            }
        }
    }

    QString currentMapName() const {
        return mapNameCombo_ ? mapNameCombo_->currentText().trimmed() : QString();
    }

    void refreshMapNameList(const QString & prefer = QString()) {
        if (!mapNameCombo_ || !dbPathEdit_) {
            return;
        }
        const QString keep = prefer.isEmpty() ? currentMapName() : prefer;
        std::vector<std::string> names;
        std::string err;
        keepout_db::listMapNames(dbPathEdit_->text().trimmed().toStdString(), names, err);
        mapNameCombo_->blockSignals(true);
        mapNameCombo_->clear();
        for (const auto & n : names) {
            mapNameCombo_->addItem(QString::fromStdString(n));
        }
        if (!keep.isEmpty()) {
            int idx = mapNameCombo_->findText(keep);
            if (idx < 0) {
                mapNameCombo_->addItem(keep);
                idx = mapNameCombo_->findText(keep);
            }
            mapNameCombo_->setCurrentIndex(idx);
            mapNameCombo_->setEditText(keep);
        }
        mapNameCombo_->blockSignals(false);
    }

    QGraphicsScene* scene_;
    QGraphicsPixmapItem* mapItem_;
    MapGraphicsView* mapView_;
    QLineEdit* gpsLatEdit_;
    QLineEdit* gpsLonEdit_;
    QLineEdit* originLatEdit_;
    QLineEdit* originLonEdit_;
    QLineEdit* originYawEdit_;
    QStackedWidget* coordModeStack_{nullptr};
    QLineEdit* pgmYamlPathEdit_{nullptr};
    QLineEdit* pgmImageEdit_{nullptr};
    QLineEdit* pgmModeEdit_{nullptr};
    QLineEdit* pgmResolutionEdit_{nullptr};
    QLineEdit* pgmOriginXEdit_{nullptr};
    QLineEdit* pgmOriginYEdit_{nullptr};
    QLineEdit* pgmOriginYawEdit_{nullptr};
    QLineEdit* pgmNegateEdit_{nullptr};
    QLineEdit* pgmOccupiedEdit_{nullptr};
    QLineEdit* pgmFreeEdit_{nullptr};
    QLineEdit* pgmSizeEdit_{nullptr};
    QLineEdit* dbPathEdit_;
    QComboBox* mapNameCombo_;
    QPushButton* refreshMapsBtn_;
    QPushButton* applyDatumBtn_;
    QPushButton* saveDbBtn_;
    QPushButton* loadDbBtn_;
    QPushButton* notifyNav2Btn_;
    QPushButton* calibrateBtn_{nullptr};
    QRadioButton* lineRadio_{nullptr};
    QRadioButton* rectRadio_{nullptr};
    QRadioButton* circleRadio_{nullptr};
    QRadioButton* polygonRadio_{nullptr};
    QToolButton* selectToolBtn_{nullptr};
    QToolButton* lineToolBtn_{nullptr};
    QToolButton* rectToolBtn_{nullptr};
    QToolButton* circleToolBtn_{nullptr};
    QToolButton* polygonToolBtn_{nullptr};
    QToolButton* chromeSelectBtn_{nullptr};
    QToolButton* chromeLineBtn_{nullptr};
    QToolButton* chromeRectBtn_{nullptr};
    QToolButton* chromeCircleBtn_{nullptr};
    QToolButton* chromePolyBtn_{nullptr};
    QButtonGroup* drawToolGroup_{nullptr};
    ShapeItem::ShapeType lastShapeTool_{ShapeItem::Line};
    QFrame* sidebarFrame_{nullptr};
    QPushButton* drawerToggleBtn_{nullptr};
    bool sidebarVisible_{true};
    QLabel* drawingToolLabel_{nullptr};
    QComboBox* mapTypeCombo_;
    QLineEdit* tileUrlEdit_;
    QPushButton* loadMapBtn_;
    QPushButton* zoomInBtn_;
    QPushButton* zoomOutBtn_;
    QPushButton* resetZoomBtn_;
    QPushButton* startDrawBtn_;
    QPushButton* stopDrawBtn_;
    QPushButton* undoDrawBtn_;
    QPushButton* clearShapesBtn_;
    QPushButton* pinBtn_;
    QPushButton* unpinBtn_;
    QPushButton* deleteSelectedBtn_;
    QCheckBox* showInflationChk_{nullptr};
    QDoubleSpinBox* inscribedRadiusSpin_{nullptr};
    QDoubleSpinBox* inflationRadiusSpin_{nullptr};
    QPushButton* publishBtn_;
    TileMapLoader* tileLoader_;
    PgmAssistController* pgmAssist_{nullptr};

    QNode* qnode_;
    geometry_msgs::msg::PoseArray prohibitionAreas_;

    // Default configuration values
    double defaultCenterLat_;
    double defaultCenterLon_;
    int defaultZoom_;
    QString defaultTileUrl_;

    // Current map state
    int currentZoom_;
    /** Zoom level of the mosaic currently on screen (may lag currentZoom_ until load finishes). */
    int displayedTileZoom_{-1};
    double currentCenterLat_;
    double currentCenterLon_;

    bool firstTileLoad_;

    // Accumulated fractional zoom delta for smooth zooming
    double zoomAccumulator_ = 0.0;

    // Current visual scale applied to the view for smooth zooming
    double currentVisualScale_ = 1.0;

    // whether the currently displayed map is a local PGM image
    bool isPgmMap_ = false;

    // Tile bounds for zoom centering
    int currentMinTileX_ = INT_MAX;
    int currentMinTileY_ = INT_MAX;

    // Debounce timer used while panning to delay tile reloads
    QTimer* panDebounceTimer_ = nullptr;
    // Debounce timer for viewport resize (fullscreen / maximize)
    QTimer* resizeDebounceTimer_ = nullptr;
    QSize lastTileViewport_;

    // PGM map metadata (from matching .yaml next to the opened .pgm)
    double mapResolution_{0.05};
    double mapOriginX_{0.0};
    double mapOriginY_{0.0};
    double mapOriginYaw_{0.0};
    int mapWidthPx_{0};
    int mapHeightPx_{0};
    QString currentPgmPath_;
    QString currentPgmYamlPath_;
    QString pgmYamlImage_;
    QString pgmYamlMode_;
    int pgmYamlNegate_{0};
    double pgmOccupiedThresh_{0.0};
    double pgmFreeThresh_{0.0};

    // Helper functions for tile coordinate conversion
    double tilex2lon(int x, int z) {
        return (x / (double)(1 << z)) * 360.0 - 180.0;
    }

    double tiley2lat(int y, int z) {
        double n = M_PI - (y / (double)(1 << z)) * 2 * M_PI;
        return atan(sinh(n)) * 180.0 / M_PI;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindowWrapper window;
    window.show();

    int result = app.exec();
    rclcpp::shutdown();
    return result;
}

#include "main.moc"
