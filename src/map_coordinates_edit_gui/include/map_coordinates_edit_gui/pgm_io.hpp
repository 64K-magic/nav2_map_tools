#ifndef MAP_COORDINATES_EDIT_GUI_PGM_IO_HPP_
#define MAP_COORDINATES_EDIT_GUI_PGM_IO_HPP_

#include <QImage>
#include <QString>

namespace pgm_io
{

/** Load PGM (P5 binary / P2 ASCII) into Format_Grayscale8. */
bool loadPGM(const QString & path, QImage & out);

/** Save QImage as binary P5 PGM (maxval 255). */
bool savePGM(const QString & path, const QImage & img);

}  // namespace pgm_io

#endif  // MAP_COORDINATES_EDIT_GUI_PGM_IO_HPP_
