#include "map_coordinates_edit_gui/pgm_io.hpp"

#include <QFile>

namespace pgm_io
{

bool loadPGM(const QString & path, QImage & out)
{
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    return false;
  }
  QByteArray data = f.readAll();
  int idx = 0;
  auto skipWs = [&]() {
    while (idx < data.size() &&
           (data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t')) {
      ++idx;
    }
  };
  skipWs();
  if (idx + 1 >= data.size() || data[idx] != 'P') {
    return false;
  }
  const char magic = data[idx + 1];
  if (magic != '5' && magic != '2') {
    return false;
  }
  idx += 2;

  auto readToken = [&](QByteArray & outTok) -> bool {
    outTok.clear();
    while (idx < data.size()) {
      if (data[idx] == '#') {
        while (idx < data.size() && data[idx] != '\n') {
          ++idx;
        }
        continue;
      }
      if (data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t') {
        ++idx;
        continue;
      }
      break;
    }
    if (idx >= data.size()) {
      return false;
    }
    while (idx < data.size() &&
           !(data[idx] == ' ' || data[idx] == '\n' || data[idx] == '\r' || data[idx] == '\t')) {
      outTok.append(data[idx]);
      ++idx;
    }
    return true;
  };

  QByteArray tok;
  bool ok = false;
  if (!readToken(tok)) {
    return false;
  }
  const int w = QString(tok).toInt(&ok);
  if (!ok || !readToken(tok)) {
    return false;
  }
  const int h = QString(tok).toInt(&ok);
  if (!ok || !readToken(tok)) {
    return false;
  }
  const int maxv = QString(tok).toInt(&ok);
  if (!ok) {
    return false;
  }
  if (idx < data.size() &&
      (data[idx] == '\n' || data[idx] == '\r' || data[idx] == ' ' || data[idx] == '\t')) {
    ++idx;
  }

  out = QImage(w, h, QImage::Format_Grayscale8);
  if (out.isNull()) {
    return false;
  }

  if (magic == '5') {
    const int samples = w * h;
    const int bytes_per_sample = (maxv < 256) ? 1 : 2;
    const qint64 expected = qint64(samples) * bytes_per_sample;
    if (data.size() - idx < expected) {
      return false;
    }
    const uchar * ptr = reinterpret_cast<const uchar *>(data.constData() + idx);
    for (int i = 0; i < samples; ++i) {
      int sample = 0;
      if (bytes_per_sample == 1) {
        sample = ptr[i];
      } else {
        sample = (ptr[2 * i] << 8) | ptr[2 * i + 1];
      }
      const int scaled = (maxv <= 255) ? sample : int((sample * 255.0) / maxv + 0.5);
      const int y = i / w;
      const int x = i % w;
      out.scanLine(y)[x] = static_cast<uchar>(qBound(0, scaled, 255));
    }
    return true;
  }

  // ASCII P2
  for (int i = 0; i < w * h; ++i) {
    if (!readToken(tok)) {
      return false;
    }
    int v = QString(tok).toInt(&ok);
    if (!ok) {
      return false;
    }
    if (maxv > 255) {
      v = int((v * 255.0) / maxv + 0.5);
    }
    const int y = i / w;
    const int x = i % w;
    out.scanLine(y)[x] = static_cast<uchar>(qBound(0, v, 255));
  }
  return true;
}

bool savePGM(const QString & path, const QImage & img)
{
  if (img.format() != QImage::Format_Grayscale8) {
    return savePGM(path, img.convertToFormat(QImage::Format_Grayscale8));
  }
  QFile f(path);
  if (!f.open(QIODevice::WriteOnly)) {
    return false;
  }
  const QString header = QStringLiteral("P5\n%1 %2\n255\n").arg(img.width()).arg(img.height());
  f.write(header.toUtf8());
  for (int y = 0; y < img.height(); ++y) {
    f.write(reinterpret_cast<const char *>(img.constScanLine(y)), img.width());
  }
  return true;
}

}  // namespace pgm_io
